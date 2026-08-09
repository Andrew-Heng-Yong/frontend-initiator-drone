import Foundation

/// A raw bidirectional message channel to rosbridge.
///
/// The client is written against this protocol rather than against
/// `URLSessionWebSocketTask` directly so the whole stack — subscriptions,
/// parsing, reconnection, rate tracking — can be exercised against the
/// fixture-driven transport with no drone and no network.
public protocol RosbridgeTransport: AnyObject {
    var delegate: RosbridgeTransportDelegate? { get set }
    var isConnected: Bool { get }

    func connect(to url: URL)
    func disconnect()
    func send(_ data: Data)
}

public protocol RosbridgeTransportDelegate: AnyObject {
    func transportDidConnect(_ transport: RosbridgeTransport)
    /// - Parameter isBinary: `true` for a WebSocket binary frame, which is how
    ///   CBOR arrives. Text frames carry JSON.
    func transport(_ transport: RosbridgeTransport, didReceive data: Data, isBinary: Bool)
    func transport(_ transport: RosbridgeTransport, didDisconnectWith error: Error?)
}

/// rosbridge over a real WebSocket.
///
/// Two details matter for a drone on field Wi-Fi:
///
/// 1. A dropped link often produces no socket error at all — the phone simply
///    stops hearing anything. The heartbeat below turns that silence into an
///    explicit disconnect within a few seconds, which is what makes automatic
///    reconnection actually fire.
/// 2. `URLSessionWebSocketTask.receive` must be re-armed after every message,
///    and exactly one call may be outstanding. The receive loop below keeps
///    that invariant on a single serial queue.
public final class WebSocketRosbridgeTransport: NSObject, RosbridgeTransport {
    public weak var delegate: RosbridgeTransportDelegate?

    /// Interval between keepalive pings.
    public var pingInterval: TimeInterval = 5.0
    /// A link with no successful pong for this long is declared dead.
    public var pongTimeout: TimeInterval = 12.0

    private let queue = DispatchQueue(label: "com.initiatordrone.rosbridge.transport")
    private var session: URLSession?
    private var task: URLSessionWebSocketTask?
    private var pingTimer: DispatchSourceTimer?
    private var lastPongAt: Date?
    private var connected = false
    private var hasReportedDisconnect = false

    public private(set) var currentURL: URL?

    public var isConnected: Bool {
        queue.sync { connected }
    }

    public override init() {
        super.init()
    }

    public func connect(to url: URL) {
        queue.async { [weak self] in
            guard let self else { return }
            self.teardownLocked(reportingError: nil, notify: false)

            self.currentURL = url
            self.hasReportedDisconnect = false

            let configuration = URLSessionConfiguration.default
            // Fail fast rather than hanging on a robot that is powered off.
            configuration.timeoutIntervalForRequest = 10
            configuration.waitsForConnectivity = false
            configuration.requestCachePolicy = .reloadIgnoringLocalAndRemoteCacheData

            let session = URLSession(configuration: configuration, delegate: self, delegateQueue: nil)
            let task = session.webSocketTask(with: url)
            // rosbridge sends whole messages; a cropped depth frame in JSON can
            // exceed the 1 MB default and would otherwise fail the socket.
            task.maximumMessageSize = 32 * 1024 * 1024

            self.session = session
            self.task = task
            self.lastPongAt = Date()
            task.resume()
            self.receiveNext()
        }
    }

    public func disconnect() {
        queue.async { [weak self] in
            self?.teardownLocked(reportingError: nil, notify: false)
        }
    }

    public func send(_ data: Data) {
        queue.async { [weak self] in
            guard let self, let task = self.task else { return }
            // Commands are JSON text; rosbridge accepts either frame type but
            // text keeps server-side logs readable.
            let message = URLSessionWebSocketTask.Message.string(
                String(data: data, encoding: .utf8) ?? ""
            )
            task.send(message) { [weak self] error in
                guard let error else { return }
                self?.queue.async { self?.teardownLocked(reportingError: error, notify: true) }
            }
        }
    }

    // MARK: - Receive loop

    private func receiveNext() {
        guard let task else { return }
        task.receive { [weak self] result in
            guard let self else { return }
            self.queue.async {
                switch result {
                case .success(let message):
                    // Any inbound traffic proves the link is alive, not just pongs.
                    self.lastPongAt = Date()
                    switch message {
                    case .data(let data):
                        self.delegate?.transport(self, didReceive: data, isBinary: true)
                    case .string(let text):
                        if let data = text.data(using: .utf8) {
                            self.delegate?.transport(self, didReceive: data, isBinary: false)
                        }
                    @unknown default:
                        break
                    }
                    self.receiveNext()

                case .failure(let error):
                    self.teardownLocked(reportingError: error, notify: true)
                }
            }
        }
    }

    // MARK: - Heartbeat

    private func startHeartbeat() {
        pingTimer?.cancel()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + pingInterval, repeating: pingInterval)
        timer.setEventHandler { [weak self] in
            guard let self else { return }

            if let lastPongAt = self.lastPongAt,
               Date().timeIntervalSince(lastPongAt) > self.pongTimeout {
                // Silent link: no error was ever delivered, so synthesise one.
                self.teardownLocked(
                    reportingError: RosbridgeTransportError.heartbeatTimeout,
                    notify: true
                )
                return
            }

            self.task?.sendPing { [weak self] error in
                guard let self else { return }
                self.queue.async {
                    if let error {
                        self.teardownLocked(reportingError: error, notify: true)
                    } else {
                        self.lastPongAt = Date()
                    }
                }
            }
        }
        timer.resume()
        pingTimer = timer
    }

    // MARK: - Teardown

    /// Must be called on `queue`.
    private func teardownLocked(reportingError error: Error?, notify: Bool) {
        pingTimer?.cancel()
        pingTimer = nil

        task?.cancel(with: .goingAway, reason: nil)
        task = nil

        session?.invalidateAndCancel()
        session = nil

        let wasConnected = connected
        connected = false

        // A failing socket can report through several paths at once; only the
        // first one should reach the client, or it will count several
        // reconnection attempts for a single drop.
        guard notify, !hasReportedDisconnect, wasConnected || error != nil else { return }
        hasReportedDisconnect = true
        delegate?.transport(self, didDisconnectWith: error)
    }
}

extension WebSocketRosbridgeTransport: URLSessionWebSocketDelegate {
    public func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didOpenWithProtocol protocolName: String?
    ) {
        queue.async { [weak self] in
            guard let self, webSocketTask === self.task else { return }
            self.connected = true
            self.lastPongAt = Date()
            self.startHeartbeat()
            self.delegate?.transportDidConnect(self)
        }
    }

    public func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didCloseWith closeCode: URLSessionWebSocketTask.CloseCode,
        reason: Data?
    ) {
        queue.async { [weak self] in
            guard let self, webSocketTask === self.task else { return }
            self.teardownLocked(
                reportingError: RosbridgeTransportError.closed(code: closeCode.rawValue),
                notify: true
            )
        }
    }

    public func urlSession(
        _ session: URLSession,
        task: URLSessionTask,
        didCompleteWithError error: Error?
    ) {
        queue.async { [weak self] in
            guard let self, task === self.task else { return }
            self.teardownLocked(
                reportingError: error ?? RosbridgeTransportError.closed(code: 0),
                notify: true
            )
        }
    }
}

public enum RosbridgeTransportError: Error, LocalizedError, Equatable {
    case heartbeatTimeout
    case closed(code: Int)
    case invalidAddress(String)

    public var errorDescription: String? {
        switch self {
        case .heartbeatTimeout:
            return "No response from rosbridge; the link went quiet."
        case .closed(let code):
            return code == 0 ? "Connection closed." : "Connection closed (code \(code))."
        case .invalidAddress(let address):
            return "'\(address)' is not a usable robot address."
        }
    }
}

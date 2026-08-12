import Foundation

/// Snapshot of `GET /api/state` from the robot's Node dashboard.
public struct DashboardState: Equatable, Sendable {
    public struct CPUCore: Equatable, Identifiable, Sendable {
        public var name: String
        public var load: Int
        public var id: String { name }
    }

    /// Whether the ROS launch process is running. Everything the robot can be
    /// asked to do depends on this.
    public var isRunning: Bool
    public var logs: [String]
    public var cpuCores: [CPUCore]
    /// Degrees Celsius, when the robot exposes a sensor.
    public var cpuTemperature: Double?
    public var receivedAt: Date

    public init(
        isRunning: Bool,
        logs: [String] = [],
        cpuCores: [CPUCore] = [],
        cpuTemperature: Double? = nil,
        receivedAt: Date = Date()
    ) {
        self.isRunning = isRunning
        self.logs = logs
        self.cpuCores = cpuCores
        self.cpuTemperature = cpuTemperature
        self.receivedAt = receivedAt
    }

    /// Parses the dashboard's JSON body.
    public static func parse(_ value: ROSValue, receivedAt: Date = Date()) -> DashboardState {
        let cores = (value["cpu"]?.arrayValue ?? []).compactMap { entry -> CPUCore? in
            guard let name = entry["core"]?.stringValue else { return nil }
            return CPUCore(name: name, load: entry["load"]?.intValue ?? 0)
        }
        return DashboardState(
            isRunning: value["running"]?.boolValue ?? false,
            logs: (value["logs"]?.arrayValue ?? []).compactMap(\.stringValue),
            cpuCores: cores,
            cpuTemperature: value["cpuTemp"]?.doubleValue,
            receivedAt: receivedAt
        )
    }
}

public enum DashboardAPIError: Error, LocalizedError, Equatable {
    case invalidEndpoint
    case httpStatus(Int)
    case malformedResponse
    case transport(String)
    /// The robot answered, but the ROS graph is stopped, so the requested
    /// action cannot mean anything.
    case graphNotRunning

    public var errorDescription: String? {
        switch self {
        case .invalidEndpoint:
            return "The robot address is not valid."
        case .httpStatus(let code):
            return code == 404
                ? "The robot dashboard does not implement this endpoint (404)."
                : "The robot dashboard returned HTTP \(code)."
        case .malformedResponse:
            return "The robot dashboard sent a response this app could not read."
        case .transport(let message):
            return message
        case .graphNotRunning:
            return "Start the robot graph before calibrating."
        }
    }
}

/// Client for the robot's dashboard HTTP API.
///
/// These four endpoints control the ROS launch process, which is why they live
/// on HTTP rather than on rosbridge: when the graph is stopped there is no
/// rosbridge to talk to.
public actor DashboardAPI {
    private let session: URLSession
    private var endpoint: RobotEndpoint?

    public init(endpoint: RobotEndpoint? = nil, session: URLSession? = nil) {
        self.endpoint = endpoint
        if let session {
            self.session = session
        } else {
            let configuration = URLSessionConfiguration.ephemeral
            configuration.timeoutIntervalForRequest = 6
            configuration.timeoutIntervalForResource = 12
            configuration.waitsForConnectivity = false
            self.session = URLSession(configuration: configuration)
        }
    }

    public func setEndpoint(_ endpoint: RobotEndpoint?) {
        self.endpoint = endpoint
    }

    // MARK: - Endpoints

    /// `GET /api/state`
    public func fetchState() async throws -> DashboardState {
        let value = try await request(path: "/api/state", method: "GET")
        guard let value else { throw DashboardAPIError.malformedResponse }
        return DashboardState.parse(value)
    }

    /// `POST /api/start`
    @discardableResult
    public func start() async throws -> Bool {
        let value = try await request(path: "/api/start", method: "POST")
        return value?["ok"]?.boolValue ?? true
    }

    /// `POST /api/stop`
    @discardableResult
    public func stop() async throws -> Bool {
        let value = try await request(path: "/api/stop", method: "POST")
        return value?["ok"]?.boolValue ?? true
    }

    /// `POST /api/odom/calibrate`, falling back to the old `/api/vio/calibrate`.
    ///
    /// Refuses to fire unless the graph is running, so the operator gets a
    /// clear reason rather than a silent no-op from a robot with no nodes up.
    ///
    /// The fallback exists because the phone app and the robot's dashboard are
    /// deployed separately: a phone updated for `odom_node` will meet robots
    /// still running the dashboard that only knows `/api/vio/calibrate`. Trying
    /// the new path first and treating a 404 — and only a 404 — as "this robot
    /// is older" keeps Calibrate working across the changeover without hiding a
    /// genuine failure, which any other status code still is.
    @discardableResult
    public func calibrateOdometry(requireRunningGraph: Bool = true) async throws -> Bool {
        if requireRunningGraph {
            let state = try await fetchState()
            guard state.isRunning else { throw DashboardAPIError.graphNotRunning }
        }
        do {
            let value = try await request(path: "/api/odom/calibrate", method: "POST")
            return value?["ok"]?.boolValue ?? true
        } catch DashboardAPIError.httpStatus(404) {
            let value = try await request(path: "/api/vio/calibrate", method: "POST")
            return value?["ok"]?.boolValue ?? true
        }
    }

    /// Round-trips `GET /api/state` purely to report reachability and latency.
    public func testConnection() async -> ConnectionTestResult {
        let started = Date()
        do {
            let state = try await fetchState()
            return ConnectionTestResult(
                isReachable: true,
                latency: Date().timeIntervalSince(started),
                graphRunning: state.isRunning,
                message: state.isRunning
                    ? "Dashboard reachable, ROS graph running."
                    : "Dashboard reachable, ROS graph stopped."
            )
        } catch {
            return ConnectionTestResult(
                isReachable: false,
                latency: Date().timeIntervalSince(started),
                graphRunning: false,
                message: (error as? LocalizedError)?.errorDescription ?? error.localizedDescription
            )
        }
    }

    // MARK: - Plumbing

    private func request(path: String, method: String) async throws -> ROSValue? {
        guard let endpoint, endpoint.isValid, let base = endpoint.dashboardBaseURL,
              let url = URL(string: path, relativeTo: base) else {
            throw DashboardAPIError.invalidEndpoint
        }

        var request = URLRequest(url: url)
        request.httpMethod = method
        request.setValue("application/json", forHTTPHeaderField: "Accept")
        request.cachePolicy = .reloadIgnoringLocalAndRemoteCacheData

        let data: Data
        let response: URLResponse
        do {
            (data, response) = try await session.data(for: request)
        } catch {
            throw DashboardAPIError.transport(error.localizedDescription)
        }

        if let http = response as? HTTPURLResponse, !(200..<300).contains(http.statusCode) {
            throw DashboardAPIError.httpStatus(http.statusCode)
        }
        guard !data.isEmpty else { return nil }
        guard let value = try? ROSValue.fromJSON(data) else {
            throw DashboardAPIError.malformedResponse
        }
        return value
    }
}

public struct ConnectionTestResult: Equatable, Sendable {
    public var isReachable: Bool
    public var latency: TimeInterval
    public var graphRunning: Bool
    public var message: String

    public init(isReachable: Bool, latency: TimeInterval, graphRunning: Bool, message: String) {
        self.isReachable = isReachable
        self.latency = latency
        self.graphRunning = graphRunning
        self.message = message
    }
}

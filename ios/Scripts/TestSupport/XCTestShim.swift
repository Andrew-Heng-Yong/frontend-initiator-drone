// A minimal stand-in for XCTest.
//
// Xcode is not required to check this project's logic: the test files below
// import XCTest only when it is available, and fall back to these definitions
// otherwise, so `Scripts/run-core-tests.sh` can compile and run the exact same
// assertions with nothing but the Swift command line tools.
//
// This file is NOT part of the Xcode test target. Inside Xcode the real XCTest
// is imported and these declarations never exist.

#if !canImport(XCTest)
import Foundation

open class XCTestCase {
    public required init() {}
    open func setUp() {}
    open func tearDown() {}
}

public struct TestFailure {
    public let message: String
    public let file: String
    public let line: UInt
}

/// Collects failures for the current test so a single test can report several.
public enum TestRecorder {
    public static var failures: [TestFailure] = []

    public static func record(_ message: String, _ file: StaticString, _ line: UInt) {
        failures.append(TestFailure(message: message, file: "\(file)", line: line))
    }
}

public func XCTFail(
    _ message: String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    TestRecorder.record(message.isEmpty ? "XCTFail" : message, file, line)
}

public func XCTAssertTrue(
    _ expression: @autoclosure () throws -> Bool,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        if try !expression() {
            TestRecorder.record(describe("expected true", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertFalse(
    _ expression: @autoclosure () throws -> Bool,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        if try expression() {
            TestRecorder.record(describe("expected false", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertEqual<T: Equatable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if a != b {
            TestRecorder.record(describe("\(a) != \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertNotEqual<T: Equatable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if a == b {
            TestRecorder.record(describe("\(a) == \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertEqual<T: FloatingPoint>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    accuracy: T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if a.isNaN || b.isNaN || abs(a - b) > accuracy {
            TestRecorder.record(describe("\(a) != \(b) (±\(accuracy))", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertNil(
    _ expression: @autoclosure () throws -> Any?,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        if let value = try expression() {
            TestRecorder.record(describe("expected nil, got \(value)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertNotNil(
    _ expression: @autoclosure () throws -> Any?,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        if try expression() == nil {
            TestRecorder.record(describe("expected non-nil", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertGreaterThan<T: Comparable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if !(a > b) {
            TestRecorder.record(describe("\(a) is not > \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertGreaterThanOrEqual<T: Comparable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if !(a >= b) {
            TestRecorder.record(describe("\(a) is not >= \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertLessThan<T: Comparable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if !(a < b) {
            TestRecorder.record(describe("\(a) is not < \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertLessThanOrEqual<T: Comparable>(
    _ lhs: @autoclosure () throws -> T,
    _ rhs: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        let a = try lhs()
        let b = try rhs()
        if !(a <= b) {
            TestRecorder.record(describe("\(a) is not <= \(b)", message()), file, line)
        }
    } catch {
        TestRecorder.record("threw \(error)", file, line)
    }
}

public func XCTAssertThrowsError<T>(
    _ expression: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line,
    _ handler: (Error) -> Void = { _ in }
) {
    do {
        _ = try expression()
        TestRecorder.record(describe("expected a thrown error", message()), file, line)
    } catch {
        handler(error)
    }
}

public func XCTAssertNoThrow<T>(
    _ expression: @autoclosure () throws -> T,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) {
    do {
        _ = try expression()
    } catch {
        TestRecorder.record(describe("threw \(error)", message()), file, line)
    }
}

public struct UnwrapFailure: Error, CustomStringConvertible {
    public let description: String
}

public func XCTUnwrap<T>(
    _ expression: @autoclosure () throws -> T?,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #filePath,
    line: UInt = #line
) throws -> T {
    guard let value = try expression() else {
        let text = describe("unexpectedly nil", message())
        TestRecorder.record(text, file, line)
        throw UnwrapFailure(description: text)
    }
    return value
}

/// A one-shot signal, matching the slice of `XCTestExpectation` these tests use.
public final class XCTestExpectation: @unchecked Sendable {
    private let lock = NSLock()
    private var fulfilledCount = 0
    public let description: String
    public var expectedFulfillmentCount = 1

    public init(description: String) {
        self.description = description
    }

    public func fulfill() {
        lock.lock()
        fulfilledCount += 1
        lock.unlock()
    }

    var isFulfilled: Bool {
        lock.lock()
        defer { lock.unlock() }
        return fulfilledCount >= expectedFulfillmentCount
    }
}

public extension XCTestCase {
    func expectation(description: String) -> XCTestExpectation {
        XCTestExpectation(description: description)
    }

    /// Spins the run loop until every expectation is met or the timeout expires.
    func wait(for expectations: [XCTestExpectation], timeout: TimeInterval) {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if expectations.allSatisfy(\.isFulfilled) { return }
            RunLoop.current.run(mode: .default, before: Date().addingTimeInterval(0.01))
        }
        for expectation in expectations where !expectation.isFulfilled {
            TestRecorder.record(
                "timed out waiting for '\(expectation.description)'",
                #filePath,
                #line
            )
        }
    }
}

private func describe(_ reason: String, _ message: String) -> String {
    message.isEmpty ? reason : "\(reason) — \(message)"
}
#endif

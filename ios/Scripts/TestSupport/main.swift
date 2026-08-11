// Entry point for running the test suite without Xcode.
//
// Compiled only by `Scripts/run-core-tests.sh`; the Xcode test target uses the
// real XCTest runner and never sees this file.

#if !canImport(XCTest)
import Foundation

struct SuiteResult {
    var suite: String
    var passed: Int
    var failed: Int
    var failures: [(test: String, failures: [TestFailure])]
}

func runSuite<T: XCTestCase>(
    _ name: String,
    _ tests: [(String, (T) -> () throws -> Void)]
) -> SuiteResult {
    var result = SuiteResult(suite: name, passed: 0, failed: 0, failures: [])

    for (testName, body) in tests {
        let instance = T()
        TestRecorder.failures = []
        instance.setUp()
        do {
            try body(instance)()
        } catch {
            TestRecorder.record("test threw \(error)", #filePath, #line)
        }
        instance.tearDown()

        if TestRecorder.failures.isEmpty {
            result.passed += 1
        } else {
            result.failed += 1
            result.failures.append((testName, TestRecorder.failures))
        }
    }
    return result
}

let started = Date()
var results: [SuiteResult] = []

results.append(runSuite("GeometryTests", GeometryTests.allTests))
results.append(runSuite("ROSImageDecoderTests", ROSImageDecoderTests.allTests))
results.append(runSuite("OdometryTests", OdometryTests.allTests))
results.append(runSuite("RosbridgeTests", RosbridgeTests.allTests))
results.append(runSuite("RenderingTests", RenderingTests.allTests))
results.append(runSuite("ConnectionTests", ConnectionTests.allTests))
results.append(runSuite("ImageStreamSoakTests", ImageStreamSoakTests.allTests))

let totalPassed = results.reduce(0) { $0 + $1.passed }
let totalFailed = results.reduce(0) { $0 + $1.failed }

print("")
for result in results {
    let status = result.failed == 0 ? "PASS" : "FAIL"
    print(String(format: "%-4@ %-24@ %3d passed, %3d failed",
                 status as NSString,
                 result.suite as NSString,
                 result.passed,
                 result.failed))
    for entry in result.failures {
        print("     ✗ \(entry.test)")
        for failure in entry.failures {
            let file = (failure.file as NSString).lastPathComponent
            print("         \(file):\(failure.line)  \(failure.message)")
        }
    }
}

let elapsed = Date().timeIntervalSince(started)
print("")
print(String(format: "%d tests, %d failed, %.2fs", totalPassed + totalFailed, totalFailed, elapsed))
exit(totalFailed == 0 ? 0 : 1)
#endif

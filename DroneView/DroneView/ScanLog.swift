import Foundation
import OSLog

/// Bounded JSONL diagnostics, deliberately excluding camera images and spatial poses.
final class ScanLog: @unchecked Sendable {
    let url: URL
    private let queue=DispatchQueue(label:"DroneView.scan-log")
    private let logger=Logger(subsystem:"randomApp.DroneView",category:"reconstruction")
    init() {
        let folder=FileManager.default.urls(for:.documentDirectory,in:.userDomainMask)[0].appendingPathComponent("ScanLogs",isDirectory:true)
        try? FileManager.default.createDirectory(at:folder,withIntermediateDirectories:true)
        url=folder.appendingPathComponent("scan-\(UUID().uuidString).jsonl")
        // Keep the latest five sessions; each file is capped at 4 MB.
        let previous=(try? FileManager.default.contentsOfDirectory(at:folder,includingPropertiesForKeys:[.creationDateKey])) ?? []
        let ordered=previous.sorted { ((try? $0.resourceValues(forKeys:[.creationDateKey]).creationDate) ?? .distantPast) > ((try? $1.resourceValues(forKeys:[.creationDateKey]).creationDate) ?? .distantPast) }
        for old in ordered.dropFirst(4) where old.pathExtension=="jsonl" {try? FileManager.default.removeItem(at:old)}
        FileManager.default.createFile(atPath:url.path,contents:nil)
    }
    func write(_ event:String,_ fields:[String:Any]=[:]) {
        var record=fields;record["event"]=event;record["time"]=Date().timeIntervalSince1970
        guard var data=try? JSONSerialization.data(withJSONObject:record,options:[.sortedKeys]) else{return}
        data.append(10);let payload=data,url=url
        logger.info("\(event,privacy:.public)")
        queue.async {
            guard let handle=try? FileHandle(forWritingTo:url) else{return}
            defer {try? handle.close()}
            guard let size=try? handle.seekToEnd(),size<4_000_000 else{return}
            try? handle.write(contentsOf:payload)
        }
    }
}

#import <Foundation/Foundation.h>
NS_ASSUME_NONNULL_BEGIN
@interface TrackingBridge : NSObject
@property(nonatomic,readonly) NSDictionary *alignmentDiagnostics;
- (void)reset;
- (NSDictionary *)processJPEG:(NSData *)jpeg depth:(NSData *)depth metadata:(NSDictionary *)metadata;
- (nullable NSData *)alignGray:(NSData *)gray width:(NSInteger)width height:(NSInteger)height depth:(NSData *)depth intrinsics:(NSArray<NSNumber *> *)intrinsics;
@end
NS_ASSUME_NONNULL_END

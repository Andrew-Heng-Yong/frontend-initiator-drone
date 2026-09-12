#include "TrackingCore.hpp"
#import "TrackingBridge.h"
using namespace drone;
static NSData *matrixData(M m) { float a[16];for(int c=0;c<4;c++)for(int r=0;r<4;r++)a[c*4+r]=float(m(r,c));return [NSData dataWithBytes:a length:sizeof(a)]; }
static cv::Matx33d intrinsic(NSArray *a) { cv::Matx33d k;for(int i=0;i<9;i++)k.val[i]=[a[i] doubleValue];return k; }
@implementation TrackingBridge { std::unique_ptr<Tracker> _tracker; NSDictionary *_alignmentDiagnostics; cv::Mat _gray; }
- (instancetype)init { if((self=[super init])){_tracker=std::make_unique<Tracker>();cv::setNumThreads(2);}return self; }
- (NSDictionary *)alignmentDiagnostics { return _alignmentDiagnostics ?: @{}; }
- (void)reset { _tracker=std::make_unique<Tracker>();_alignmentDiagnostics=nil;_gray.release(); }
- (NSDictionary *)processJPEG:(NSData *)jpeg depth:(NSData *)depth metadata:(NSDictionary *)m {
    @try { try {
        int w=[m[@"width"] intValue],h=[m[@"height"] intValue];
        if(w<=0||h<=0||w>1920||h>1080||depth.length!=size_t(w)*h*4||[m[@"K"] count]!=9) return @{@"status":@"invalid"};
        cv::Mat bytes(1,int(jpeg.length),CV_8U,const_cast<void *>(jpeg.bytes));auto image=cv::imdecode(bytes,cv::IMREAD_COLOR);
        if(!image.empty())cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);
        if(image.cols!=w||image.rows!=h)return @{@"status":@"invalid"};
        _gray=image;
        cv::Mat d(h,w,CV_32F,const_cast<void *>(depth.bytes));auto f=frame(image,d,intrinsic(m[@"K"]),[m[@"timestamp"] doubleValue]);
        cv::Matx33d increment=cv::Matx33d::eye();bool ready=false;
        NSDictionary *g=m[@"gyro"];NSArray *samples=g[@"samples"],*bias=g[@"bias"],*mount=g[@"rotation"];
        double start=_tracker->lastGood,end=f.stamp;
        if([g[@"ready"] boolValue]&&start>0&&end>start&&end-start<=1&&samples.count>=2&&bias.count==3&&mount.count==3) {
            cv::Matx33d mounting;for(int r=0;r<3;r++)for(int c=0;c<3;c++)mounting(r,c)=[mount[r][c] doubleValue];
            std::vector<GyroSample> history;
            for(NSArray *row in samples) {
                if(row.count!=4)return @{@"status":@"invalid"};
                history.push_back({[row[0] doubleValue],{[row[1] doubleValue],[row[2] doubleValue],[row[3] doubleValue]}});
            }
            auto integrated=integrateGyro(history,start,end,{[bias[0] doubleValue],[bias[1] doubleValue],[bias[2] doubleValue]},mounting);
            if(integrated){increment=*integrated;ready=true;}

        }
        auto fit=_tracker->update(f,ready?&increment:nullptr);
        return @{@"status":_tracker->status==1?@"tracking":_tracker->status==2?@"lost":@"initializing",
                 @"pose":matrixData(_tracker->pose),@"inliers":@(fit.inliers),@"gyro":@(ready),@"keyframes":@(_tracker->keyframes.size())};
    }catch(const std::exception &){return @{@"status":@"invalid"};}}
    @catch(NSException *e){return @{@"status":@"invalid"};}
}
- (NSData *)alignGray:(NSData *)gray width:(NSInteger)w height:(NSInteger)h depth:(NSData *)depth intrinsics:(NSArray<NSNumber *> *)k {
    if(!_tracker->initialized||_gray.empty()||w<=0||h<=0||w>1920||h>1440||gray.length!=size_t(w*h)||depth.length!=size_t(w*h*4)||k.count!=9)return nil;
    try {
        auto start=cv::getTickCount();
        auto rig=frame(_gray,_tracker->current.depth,_tracker->current.k,0,true);
        auto phone=frame(cv::Mat(int(h),int(w),CV_8U,const_cast<void *>(gray.bytes)),cv::Mat(int(h),int(w),CV_32F,const_cast<void *>(depth.bytes)),intrinsic(k),0,true);
        auto match=fit(rig,phone,nullptr,true);
        _alignmentDiagnostics=@{@"rig_features":@(rig.keys.size()),@"phone_features":@(phone.keys.size()),
                                @"processing_ms":@(int(1000*(cv::getTickCount()-start)/cv::getTickFrequency())),
                                @"matches":@(match.matches),@"inliers":@(match.inliers),@"depth_samples":@(match.depthSamples),@"accepted":@(match.valid&&match.inliers>=30),
                                @"rejection":@(match.valid&&match.inliers<30?10:match.rejection),
                                @"depth_median_mm":@(int(match.depthMedian*1000)),@"depth_p75_mm":@(int(match.depthP75*1000))};
        // phone optical -> rig optical; caller uses both capture-time poses.
        return match.valid&&match.inliers>=30?matrixData(match.pose):nil;
    }catch(const std::exception &){return nil;}
}
@end

#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <map>
#include <optional>
#include <cmath>

// PnP port of tracking/odometry.py. All transforms are camera-to-reference.
namespace drone {
using M = cv::Matx44d;
inline cv::Matx33d rotation(const M &m) { return m.get_minor<3,3>(0,0); }
inline double angle(cv::Matx33d r) { return std::acos(std::clamp((cv::trace(r)-1)*.5,-1.,1.)); }
inline cv::Vec3d translation(const M &m) { return {m(0,3),m(1,3),m(2,3)}; }
inline double quantile(std::vector<double> a, double q) {
    std::sort(a.begin(),a.end()); double x=(a.size()-1)*q;
    int i=int(x); return a[i]+(a[std::min(i+1,int(a.size()-1))]-a[i])*(x-i);
}
inline double depthAt(const cv::Mat &d, cv::Point2f p) {
    int x=int(std::nearbyint(p.x)), y=int(std::nearbyint(p.y));
    if(x<0||y<0||x>=d.cols||y>=d.rows) return NAN;
    std::vector<double> a;
    for(int v=std::max(0,y-1);v<std::min(d.rows,y+2);++v)
        for(int u=std::max(0,x-1);u<std::min(d.cols,x+2);++u) {
            double z=d.at<float>(v,u); if(std::isfinite(z)&&z>=.2&&z<=6) a.push_back(z);
        }
    return a.empty()?NAN:quantile(a,.5);
}
inline cv::Point3f backproject(cv::Point2f p,double z,const cv::Matx33d &k) {
    return {float((p.x-k(0,2))*z/k(0,0)),float((p.y-k(1,2))*z/k(1,1)),float(z)};
}
struct GyroSample { double time;cv::Vec3d rate; };
inline std::optional<cv::Matx33d> integrateGyro(const std::vector<GyroSample> &samples,
        double start,double end,cv::Vec3d bias,cv::Matx33d mount) {
    if(!std::isfinite(start)||!std::isfinite(end)||start<=0||end<=start||end-start>1||samples.size()<2
       ||!cv::checkRange(cv::Mat(bias))||!cv::checkRange(cv::Mat(mount)))return {};
    cv::Matx33d result=cv::Matx33d::eye();double covered=start;
    for(size_t i=1;i<samples.size();i++) {
        auto a=samples[i-1],b=samples[i];
        if(!std::isfinite(a.time)||!std::isfinite(b.time)||b.time<=a.time
           ||!cv::checkRange(cv::Mat(a.rate))||!cv::checkRange(cv::Mat(b.rate)))return {};
        if(b.time<=start||a.time>=end)continue;
        if(b.time-a.time>.25||a.time>covered+.000001)return {};
        double lo=std::max(start,a.time),hi=std::min(end,b.time);
        cv::Vec3d rate=a.rate+(b.rate-a.rate)*(((lo+hi)*.5-a.time)/(b.time-a.time))-bias;
        cv::Matx33d step;cv::Rodrigues(mount*rate*(hi-lo),step);result=result*step;covered=hi;
    }
    if(covered<end-.000001)return {};return result;
}
struct Frame {
    cv::Mat depth, descriptors;
    std::vector<cv::KeyPoint> keys;
    cv::Matx33d k;
    M pose=M::eye(); double stamp=0;
};
inline Frame frame(const cv::Mat &image,const cv::Mat &depth,cv::Matx33d k,double stamp) {
    Frame f; f.depth=depth.clone();f.k=k;f.stamp=stamp;
    auto orb=cv::ORB::create(800,1.2f,8,15,0,2,cv::ORB::HARRIS_SCORE,31,10);
    orb->detectAndCompute(image,cv::noArray(),f.keys,f.descriptors);return f;
}
struct Fit { bool valid=false; M pose=M::eye(); int inliers=0, matches=0, depthSamples=0, rejection=1; double depthMedian=0, depthP75=0; double rmse=0; };
inline Fit fit(const Frame &ref,const Frame &cur,const cv::Matx33d *prior=nullptr,bool requireDepth=false) {
    Fit out;
    if(ref.descriptors.rows<2||cur.descriptors.rows<2) return out;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> forward,reverse;
    matcher.knnMatch(ref.descriptors,cur.descriptors,forward,2);
    matcher.knnMatch(cur.descriptors,ref.descriptors,reverse,2);
    auto unique=[](auto &pairs) {
        std::map<int,cv::DMatch> result;
        for(auto &p:pairs) if(p.size()==2&&p[1].distance>0&&p[0].distance<.8*p[1].distance) {
            auto it=result.find(p[0].trainIdx);
            if(it==result.end()||p[0].distance<it->second.distance) result[p[0].trainIdx]=p[0];
        } return result;
    };
    auto a=unique(forward), b=unique(reverse); std::vector<cv::DMatch> matches;
    for(auto &[i,m]:a) if(b.count(m.queryIdx)&&b[m.queryIdx].queryIdx==m.trainIdx) matches.push_back(m);
    if(matches.size()<4) { matches.clear();for(auto &[i,m]:a) matches.push_back(m); }
    std::sort(matches.begin(),matches.end(),[](auto a,auto b){return a.distance==b.distance?a.queryIdx<b.queryIdx:a.distance<b.distance;});
    out.matches=int(matches.size()); std::vector<cv::Point3f> objects;std::vector<cv::Point2f> pixels;
    for(auto &m:matches) { auto p=ref.keys[m.queryIdx].pt;double z=depthAt(ref.depth,p);
        if(std::isfinite(z)){objects.push_back(backproject(p,z,ref.k));pixels.push_back(cur.keys[m.trainIdx].pt);}}
    out.rejection=2;if(objects.size()<4)return out;
    auto spread=[](const std::vector<cv::Point3f> &p) {
        cv::Mat m(int(p.size()),3,CV_64F);cv::Vec3d mean(0,0,0);
        for(auto v:p) mean+=cv::Vec3d(v.x,v.y,v.z);mean/=double(p.size());
        for(int i=0;i<int(p.size());i++)for(int j=0;j<3;j++)m.at<double>(i,j)=cv::Vec3d(p[i].x,p[i].y,p[i].z)[j]-mean[j];
        cv::Mat w;cv::SVD::compute(m,w);return w.at<double>(1);
    };
    out.rejection=3;if(spread(objects)<=1e-9)return out;
    // Retry without gyro exactly when the seeded fit fails its acceptance gates.
    for(int attempt=0;attempt<(prior?2:1);attempt++) {
        bool seeded=prior&&attempt==0;cv::Mat rvec,tvec=cv::Mat::zeros(3,1,CV_64F),ids;
        if(seeded)cv::Rodrigues(prior->t(),rvec);
        try {
            cv::setRNGSeed(7);
            out.rejection=4;
            if(!cv::solvePnPRansac(objects,pixels,cur.k,cv::noArray(),rvec,tvec,seeded,100,3.,.99,ids,
                                  seeded?cv::SOLVEPNP_ITERATIVE:cv::SOLVEPNP_EPNP)||ids.rows<4)continue;
            std::vector<cv::Point3f> oi;std::vector<cv::Point2f> pi;
            for(int i=0;i<ids.rows;i++){int j=ids.at<int>(i);oi.push_back(objects[j]);pi.push_back(pixels[j]);}
            cv::solvePnPRefineLM(oi,pi,cur.k,cv::noArray(),rvec,tvec,{cv::TermCriteria::EPS+cv::TermCriteria::COUNT,20,1e-6});
            cv::Matx33d r;cv::Rodrigues(rvec,r);cv::Vec3d t(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));
            std::vector<cv::Point2f> projected;cv::projectPoints(objects,rvec,tvec,cur.k,cv::noArray(),projected);
            std::vector<cv::Point3f> good;std::vector<int> indices;double square=0;bool behind=false;
            for(int i=0;i<int(objects.size());i++)if(cv::norm(projected[i]-pixels[i])<=3.) {
                good.push_back(objects[i]);indices.push_back(i);square+=std::pow(cv::norm(projected[i]-pixels[i]),2);
                auto p=r*cv::Vec3d(objects[i].x,objects[i].y,objects[i].z)+t;if(!cv::checkRange(cv::Mat(p))||p[2]<=.1)behind=true;
            }
            out.inliers=int(good.size());
            double ratio=double(good.size())/objects.size();
            out.rejection=5;
            if(behind||good.size()<12||ratio<.3||spread(good)<.02)continue;
            cv::Matx33d inverse=r.t();cv::Vec3d shift=-(inverse*t);
            std::vector<double> residuals;
            int n=std::min(128,int(indices.size()));
            for(int j=0;j<n;j++) {
                int i=indices[n==1?0:int(double(j)*(indices.size()-1)/(n-1))];double z=depthAt(cur.depth,pixels[i]);
                if(!std::isfinite(z))continue;
                auto p=backproject(pixels[i],z,cur.k);auto q=objects[i];
                residuals.push_back(cv::norm(inverse*cv::Vec3d(p.x,p.y,p.z)+shift-cv::Vec3d(q.x,q.y,q.z)));
            }
            out.depthSamples=int(residuals.size());
            out.rejection=6;
            if(requireDepth&&residuals.size()<12)continue;
            if(!residuals.empty()){out.depthMedian=quantile(residuals,.5);out.depthP75=quantile(residuals,.75);}
            out.rejection=7;
            if(residuals.size()>=8&&(out.depthMedian>.1||out.depthP75>.2))continue;
            out.rejection=8;
            if(seeded&&angle(prior->t()*inverse)>CV_PI/3&&ratio<.75)continue;
            out.rejection=9;
            if(!cv::checkRange(cv::Mat(inverse))||cv::determinant(inverse)<=0)continue;
            out.pose=M::eye();for(int y=0;y<3;y++){for(int x=0;x<3;x++)out.pose(y,x)=inverse(y,x);out.pose(y,3)=shift[y];}
            out.valid=true;out.rejection=0;out.inliers=int(good.size());out.depthSamples=int(residuals.size());out.rmse=std::sqrt(square/good.size());return out;
        } catch(const cv::Exception &) { continue; }
    }return out;
}
class Tracker {
public:
    Frame active,origin,recovery,current;M pose=M::eye();double lastGood=0,last=0;bool initialized=false,activeOrigin=true;int status=0;
    Fit update(Frame f,const cv::Matx33d *increment=nullptr) {
        Fit result;
        if(!std::isfinite(f.stamp)||f.stamp<=last)return result;last=f.stamp;current=f;
        if(!initialized||(status==0&&active.descriptors.rows<4)) {
            active=origin=f;lastGood=f.stamp;initialized=true;return result;
        }
        cv::Matx33d prior=increment?rotation(active.pose).t()*rotation(pose)*(*increment):cv::Matx33d::eye();
        result=fit(active,f,increment?&prior:nullptr);M base=active.pose;bool recovered=false;
        if(!result.valid&&recovery.stamp!=active.stamp) {
            result=fit(recovery,f);if(result.inliers<50)result.valid=false;
            if(result.valid){base=recovery.pose;recovered=true;}
        }
        if(!result.valid&&!activeOrigin){result=fit(origin,f);if(result.valid){base=origin.pose;recovered=true;}}
        if(!result.valid){status=2;return result;}
        M next=base*result.pose;double dt=f.stamp-lastGood;
        if(dt<=0||cv::norm(translation(next)-translation(pose))/dt>5||angle(rotation(pose).t()*rotation(next))/dt>3){result.valid=false;status=2;return result;}
        pose=next;f.pose=pose;current.pose=pose;lastGood=f.stamp;status=1;
        if(recovered||cv::norm(translation(result.pose))>=.2||angle(rotation(result.pose))>=CV_PI/12){active=f;activeOrigin=false;}
        recovery=f;return result;
    }
};
}

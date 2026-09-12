#include "../DroneView/DroneView/TrackingCore.hpp"
#include <fstream>
#include <iomanip>
#include <cassert>
#include <iostream>
static drone::Frame roomView(double x,double stamp) {
    drone::Frame f;f.stamp=stamp;f.k={500,0,320,0,500,180,0,0,1};
    f.depth=cv::Mat(360,640,CV_32F,cv::Scalar(2));
    for(int ix=-10;ix<180;ix++)for(int iy=-5;iy<=5;iy++) {
        float u=float(500*(ix*.1-x)/2+320),v=float(500*iy*.1/2+180);
        if(u<20||u>620||v<20||v>340)continue;
        f.keys.emplace_back(cv::Point2f(u,v),10);
        cv::Mat descriptor(1,32,CV_8U);cv::RNG random(1+(ix+10)*11+iy+5);
        random.fill(descriptor,cv::RNG::UNIFORM,0,256);f.descriptors.push_back(descriptor);
    }
    return f;
}
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    if(std::string(argv[1])=="--check") {
        std::vector<drone::GyroSample> samples={{1,{.2,0,0}},{1.1,{.2,0,0}},{1.2,{.2,0,0}}};
        auto r=drone::integrateGyro(samples,1.025,1.175,{.1,0,0},cv::Matx33d::eye());
        assert(r && std::abs(drone::angle(*r)-.015)<1e-8);
        assert(!drone::integrateGyro(samples,.9,1.1,{0,0,0},cv::Matx33d::eye()));
        assert(!drone::integrateGyro(samples,1,2.1,{0,0,0},cv::Matx33d::eye()));
        samples[1].time=2;
        assert(!drone::integrateGyro(samples,1,1.2,{0,0,0},cv::Matx33d::eye()));
        cv::setNumThreads(2);
        cv::Mat image(480,640,CV_8U,cv::Scalar(210)),depth(480,640,CV_32F,cv::Scalar(2));
        cv::RNG random(19);
        for(int i=0;i<250;i++) {
            cv::Point p(random.uniform(15,605),random.uniform(20,460));
            cv::putText(image,std::to_string(i),p,cv::FONT_HERSHEY_SIMPLEX,random.uniform(.3,.8),cv::Scalar(random.uniform(0,160)),1,cv::LINE_AA);
        }
        cv::Matx33d k(500,0,320,0,500,240,0,0,1),phoneK(570,0,320,0,570,240,0,0,1);
        for(double baseline:{.15,.4,.65}) {
            cv::Matx33d rotation;cv::Rodrigues(cv::Vec3d(0,-baseline/2,0),rotation);
            cv::Vec3d shift(baseline,.02,.03),inverseShift=-(rotation.t()*shift);
            cv::Matx33d homography=phoneK*(rotation.t()+inverseShift*cv::Vec3d(0,0,.5).t())*k.inv();
            cv::Mat phone,phoneDepth(depth.size(),CV_32F);
            cv::warpPerspective(image,phone,homography,image.size());phone.convertTo(phone,CV_8U,.65,35);
            for(int y=0;y<phone.rows;y++)for(int x=0;x<phone.cols;x++)
                phoneDepth.at<float>(y,x)=float((2-shift[2])/(rotation(2,0)*(x-phoneK(0,2))/phoneK(0,0)+rotation(2,1)*(y-phoneK(1,2))/phoneK(1,1)+rotation(2,2)));
            auto start=cv::getTickCount();
            auto rig=drone::frame(image,depth,k,1,true),other=drone::frame(phone,phoneDepth,phoneK,1,true);
            auto aligned=drone::fit(rig,other,nullptr,true);
            double elapsed=1000*(cv::getTickCount()-start)/cv::getTickFrequency();
            std::cout<<"Baseline "<<baseline<<" m: SIFT "<<aligned.inliers<<" inliers, "<<elapsed<<" ms\n";
            assert(aligned.valid&&aligned.inliers>=30);
            assert(cv::norm(drone::translation(aligned.pose)-shift)<.02);
            assert(drone::angle(drone::rotation(aligned.pose).t()*rotation)<.02);
            auto orb=drone::fit(drone::frame(image,depth,k,1),drone::frame(phone,phoneDepth,phoneK,1),nullptr,true);
            std::cout<<"ORB comparison: "<<orb.inliers<<" inliers\n";
            other.depth=phoneDepth+.6f;
            assert(!drone::fit(rig,other,nullptr,true).valid);
            other.depth=cv::Mat(phoneDepth.size(),CV_32F,cv::Scalar(NAN));
            assert(!drone::fit(rig,other,nullptr,true).valid);
        }
        assert(!drone::fit(drone::frame(image,depth,k,1),drone::frame(image,depth,k,1,true)).valid);
        drone::Tracker tracker;
        for(int i=0;i<=40;i++) {
            auto fit=tracker.update(roomView(i*.3,1+i*.2));
            if(i>0){assert(fit.valid);assert(std::abs(tracker.pose(0,3)-i*.3)<.01);}
            assert(tracker.keyframes.size()<=24);
        }
        assert(tracker.keyframes.size()==24);
        auto lastPose=tracker.pose;
        auto unseen=roomView(100,10);assert(!tracker.update(unseen).valid);assert(tracker.pose==lastPose);
        auto revisit=roomView(8.4,11);
        assert(!drone::fit(tracker.origin,revisit).valid);
        assert(!drone::fit(tracker.active,revisit).valid);
        assert(!drone::fit(tracker.recovery,revisit).valid);
        auto inconsistent=revisit;inconsistent.depth=cv::Mat(revisit.depth+.6f);
        assert(revisit.depth.at<float>(0,0)==2);
        for(int i=0;i<6;i++){inconsistent.stamp=11+i*.2;assert(!tracker.update(inconsistent).valid);}
        bool recovered=false;
        auto start=cv::getTickCount();
        for(int i=0;i<6&&!recovered;i++){revisit.stamp=13+i*.2;recovered=tracker.update(revisit).valid;}
        assert(recovered);assert(std::abs(tracker.pose(0,3)-8.4)<.01);
        assert(drone::angle(drone::rotation(tracker.pose))<.01);
        std::cout<<"Keyframe revisit recovered in "<<1000*(cv::getTickCount()-start)/cv::getTickFrequency()<<" ms; bad depth rejected\n";
        tracker=drone::Tracker();assert(tracker.keyframes.empty());assert(!tracker.initialized);
        std::cout<<"Gyro, cross-camera and bounded keyframe recovery checks passed\n";return 0;
    }
    cv::FileStorage file(argv[1],cv::FileStorage::READ);drone::Tracker tracker;
    cv::setNumThreads(2);
    for(auto entry:file["frames"]) {
        std::string imagePath,depthPath;double stamp;cv::Mat k;
        entry["image"]>>imagePath;entry["depth"]>>depthPath;entry["stamp"]>>stamp;entry["K"]>>k;
        auto image=cv::imread(imagePath,cv::IMREAD_COLOR);cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);cv::Mat depth(image.rows,image.cols,CV_32F);
        std::ifstream input(depthPath,std::ios::binary);input.read((char*)depth.data,depth.total()*4);
        if(!input||k.total()!=9)return 3;
        cv::Matx33d intrinsic;std::copy(k.ptr<double>(),k.ptr<double>()+9,intrinsic.val);
        auto fit=tracker.update(drone::frame(image,depth,intrinsic,stamp));
        std::cout<<tracker.status;for(double x:tracker.pose.val)std::cout<<","<<std::setprecision(12)<<x;std::cout<<"\n";
    }
}

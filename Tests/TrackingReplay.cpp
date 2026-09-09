#include "../DroneView/DroneView/TrackingCore.hpp"
#include <fstream>
#include <iomanip>
#include <cassert>
#include <iostream>
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
        std::cout<<"Gyro interpolation, bias, bounds and gaps passed\n";return 0;
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

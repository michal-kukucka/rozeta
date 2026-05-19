#include <rozeta/lidar.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
namespace rozeta::lidar {
std::vector<ScanPoint> filterInvalid(const std::vector<ScanPoint>& points,double min_m,double max_m){ std::vector<ScanPoint> out; for(auto p:points) if(p.valid && p.distance_m>=min_m && p.distance_m<=max_m) out.push_back(p); return out; }
std::string renderConsoleScan(const std::vector<ScanPoint>& points,int columns,double max_m){ std::string line(columns,' '); int mid=columns/2; for(auto p:filterInvalid(points,0.01,max_m)){ if(p.angle_deg<-90||p.angle_deg>90) continue; int idx=mid + int((p.angle_deg/90.0)*mid); if(idx>=0&&idx<columns) line[idx]=(p.distance_m<1.0?'#':'.'); } return line; }
Status MockLidarScanner::initialize(const std::string&){ return Status::okStatus(); }
Status MockLidarScanner::start(){ running_=true; return Status::okStatus(); }
Status MockLidarScanner::stop(){ running_=false; return Status::okStatus(); }
Scan MockLidarScanner::readScan(){ return running_?scan_:Scan{}; }
void MockLidarScanner::setScan(Scan s){ scan_=std::move(s); }
}
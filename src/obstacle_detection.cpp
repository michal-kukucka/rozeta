#include <rozeta/obstacle_detection.hpp>
#include <algorithm>
#include <limits>
namespace rozeta::obstacle_detection {
ObstacleInfo fromLidar(const std::vector<lidar::ScanPoint>& scan,double threshold){ ObstacleInfo info; info.nearestDistance=std::numeric_limits<double>::infinity(); for(auto p:scan){ if(!p.valid) continue; info.nearestDistance=std::min(info.nearestDistance,p.distance_m); if(p.distance_m<=threshold){ if(p.angle_deg>=-25 && p.angle_deg<=25) info.obstacleAhead=true; if(p.angle_deg<-25 && p.angle_deg>=-100) info.obstacleLeft=true; if(p.angle_deg>25 && p.angle_deg<=100) info.obstacleRight=true; } } if(info.nearestDistance==std::numeric_limits<double>::infinity()) info.nearestDistance=0; return info; }
ObstacleInfo combine(const std::vector<ObstacleInfo>& inputs){ ObstacleInfo out; out.nearestDistance=std::numeric_limits<double>::infinity(); for(auto i:inputs){ out.obstacleAhead|=i.obstacleAhead; out.obstacleLeft|=i.obstacleLeft; out.obstacleRight|=i.obstacleRight; if(i.nearestDistance>0) out.nearestDistance=std::min(out.nearestDistance,i.nearestDistance); } if(out.nearestDistance==std::numeric_limits<double>::infinity()) out.nearestDistance=0; return out; }
}
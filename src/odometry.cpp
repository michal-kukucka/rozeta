#include <rozeta/odometry.hpp>
#include <cmath>
namespace rozeta::odometry {
DifferentialOdometry::DifferentialOdometry(DifferentialDriveConfig c):config_(c){}
Pose2D DifferentialOdometry::updateTicks(std::int64_t l,std::int64_t r){ if(!have_last_){ last_left_=0; last_right_=0; have_last_=true; } auto dl_ticks=l-last_left_; auto dr_ticks=r-last_right_; last_left_=l; last_right_=r; double meters_per_tick=(2*M_PI*config_.wheel_radius_m)/config_.ticks_per_wheel_revolution; double dl=dl_ticks*meters_per_tick, dr=dr_ticks*meters_per_tick; double dc=(dl+dr)/2.0; double dh=(dl-dr)/config_.wheel_base_m; pose_.x += dc*std::cos(pose_.heading + dh/2.0); pose_.y += dc*std::sin(pose_.heading + dh/2.0); pose_.heading=normalizeAngle(pose_.heading+dh); distance_ += std::fabs(dc); return pose_; }
void DifferentialOdometry::reset(Pose2D p){ pose_=p; last_left_=last_right_=0; have_last_=false; distance_=0; }
Pose2D DifferentialOdometry::pose() const { return pose_; }
double DifferentialOdometry::distanceTravelled() const { return distance_; }
}
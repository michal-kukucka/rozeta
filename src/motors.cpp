#include <rozeta/motors.hpp>
#include <cmath>
namespace rozeta::motors {
static Direction dir(double v){ return v>0?Direction::Forward:(v<0?Direction::Reverse:Direction::Stopped); }
MockMotorController::MockMotorController(MotorCalibration c):calibration_(c){}
Status MockMotorController::setSpeed(double l,double r){ if(emergency_) return Status::error(ErrorCode::EmergencyStopped,"emergency stop active"); if(std::fabs(l)>calibration_.max_speed||std::fabs(r)>calibration_.max_speed) return Status::error(ErrorCode::InvalidArgument,"speed outside calibrated range"); last_={l*calibration_.left_scale,r*calibration_.right_scale,dir(l),dir(r)}; return Status::okStatus(); }
Status MockMotorController::stop(){ last_={}; return Status::okStatus(); }
void MockMotorController::emergencyStop(){ emergency_=true; last_={}; }
void MockMotorController::clearEmergencyStop(){ emergency_=false; }
bool MockMotorController::isEmergencyStopped() const { return emergency_; }
EncoderFeedback MockMotorController::encoderFeedback() const { return feedback_; }
void MockMotorController::setEncoderFeedback(EncoderFeedback f){ feedback_=f; }
MotorCommand MockMotorController::lastCommand() const { return last_; }
}
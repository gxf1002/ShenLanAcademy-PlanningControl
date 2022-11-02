#include "carla_shenlan_pid_controller/pid_controller.h"

#include <assert.h>
#include <iostream>
#include <math.h>
using namespace std;
namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    error_sum = 0.0;
    error_sub = 0.0;
    differential_=0.0;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
    assert(dt > 0 && "dt must be positive!!!");
   if(fabs(integral_)>5){
        PIDController::Reset();
   }   
    error_sum = error_sum + error;  //误差求和
    error_sub=error-previous_error_; //误差求差
   integral_ = integral_ + dt*error; //积分环节的实现
   if (first_hit_){
    first_hit_= false;
   }
   else{
       //微分环节的实现
   differential_=error_sub/dt;
   }
   current_output = kp_*error + ki_*integral_ +kd_*differential_;  //PID控制器的实现
   //更新误差
   previous_error_ = error;
   cout<<"速度误差："<<error;
   cout<<"积分和："<<integral_;
   return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {

    integral_ =0;
    current_output =0;
    previous_error_ =0 ;
    first_hit_ = true;  
}

}    // namespace control
}    // namespace shenlan

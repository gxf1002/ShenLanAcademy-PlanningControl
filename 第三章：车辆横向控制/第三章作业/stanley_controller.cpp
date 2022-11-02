#include "carla_shenlan_stanley_pid_controller/stanely_controller.h"
#include "carla_shenlan_stanley_pid_controller/pid_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

namespace shenlan {
    namespace control {

        shenlan::control::PIDController e_theta_pid_controller(1, 0.0, 0.4); // PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上

        double atan2_to_PI(const double atan2) 
        {
            return atan2 * M_PI / 180;
        }

        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) 
        {
            const double dx = point.x - x;
            const double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        void StanleyController::LoadControlConf() 
        {
            k_y_ = 1;
        }

        // /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
        // 提示，在该函数中你需要调用计算误差
        // 控制器中，前轮转角的命令是弧度单位，发送给carla的横向控制指令，范围是 -1~1
        void StanleyController::ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) 
        {  

           double vehicle_x = vehicle_state.x;
           
           double vehicle_y = vehicle_state.y;
                                                                                                                  
           double vehicle_theta = vehicle_state.heading;
           double vehicle_vel = vehicle_state.v+0.001;
           double e_y;
           double e_theta;
           if(trajectory_points_.empty())
           {
             trajectory_points_= planning_published_trajectory.trajectory_points;
           }
           ComputeLateralErrors(vehicle_x,vehicle_y,vehicle_theta,e_y,e_theta);
           cout<<"自车角度：" <<vehicle_theta<<endl;
           cout<<"自车速度：" <<vehicle_state.v;
           double target_cross= e_theta+atan2(k_y_*e_y,vehicle_vel);
           if(target_cross > M_PI/3) 
           {
               target_cross=  M_PI / 3 ;
           }
           else if (target_cross < -M_PI/3)
           {
               target_cross =  -M_PI / 3;
           }
           cout<<"e_theta="<<e_theta<<endl;

           cmd.steer_target=target_cross;
           cout << " 方向盘转动角度： "<<cmd.steer_target<<endl;
        
        // cmd.steer_target=e_theta+atan2(k*e_y,vehicle_vel);
           
            
        }

        // /** to-do **/ 计算需要的误差，包括横向误差，纵向误差，误差计算函数没有传入主车速度，因此返回的位置误差就是误差，不是根据误差计算得到的前轮转角
        void StanleyController::ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta) 
        {
           

            auto trajectory_points = QueryNearestPointByPosition(x,y);
            double dx = trajectory_points.x-x;
            double dy = trajectory_points.y-y;
            cout<<"参考点角度:"<<trajectory_points.heading<<endl;
            cout<<"参考点的速度:"<<trajectory_points.v<<endl;
            double delta = trajectory_points.heading-atan2(dy,dx);
            e_y = sin(delta)*PointDistanceSquare(trajectory_points, x, y);
            cout << "横向误差："<< e_y <<endl;
            e_theta = theta-trajectory_points.heading;
            if(e_theta > M_PI)
            {
                e_theta =e_theta- 2 * M_PI ;
            }
            else if (e_theta < -M_PI)
            {
               
               e_theta = e_theta + 2 * M_PI ;
            }


        }

        // 返回参考路径上和车辆当前位置距离最近的点，返回的是点结构体
        TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x, const double y) 
        {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y); // 得到当前位置距离参考路径的第一个点的距离
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i) 
            {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min) 
                {
                    d_min = d_temp;
                    index_min = i;
                }
            }
            // cout << " index_min: " << index_min << endl;
            //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
            theta_ref_ = trajectory_points_[index_min].heading; // 获得距离车辆当前位置最近的路径点的航向角

            return trajectory_points_[index_min];
        }
    }  // namespace control
}  // namespace shenlan

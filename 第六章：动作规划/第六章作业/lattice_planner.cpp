// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "carla_shenlan_lattice_planner/lattice_planner.h"

#include <algorithm>
#include <cmath>

// #include "ros/ros.h"

// namespace shenlan {
#define MAX_SPEED 50.0       // maximum speed [m/s]
#define MAX_ACCEL 6.0              // maximum acceleration [m/ss]
#define MAX_CURVATURE 100.0          // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0         // maximum road width [m]
#define D_ROAD_W 1.0               // road width sampling length [m]
#define DT 0.2                     // time tick [s]
#define MAXT 3.0                   // max prediction time [s]
#define MINT 2.0                   // min prediction time [s]
#define TARGET_SPEED 17.0 / 3.6    // target speed [m/s]
#define D_T_S 5.0 / 3.6            // target speed sampling length [m/s]
#define N_S_SAMPLE 1               // sampling number of target speed
#define ROBOT_RADIUS 2.5           // robot radius [m]

#define KJ 1
#define KT 0.1
#define KD 1.4
#define KLAT 1.0
#define KLON 1.0

FrenetOptimalTrajectory::FrenetOptimalTrajectory() {}
FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list) {
    float sum = 0;
    for (float item : value_list) {
        sum += item * item;
    }
    return sum;
};

// 01 获取采样轨迹
Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d, float c_d_d, float c_d_dd, float s0) {
    std::vector<FrenetPath> fp_list;
    // 根据 当前车辆的速度，当前车辆在frenet坐标系中的s坐标，l坐标，l坐标的一阶导数和二阶导数来采样生成备选轨迹
    // 先遍历 d 方向，再遍历 t 方向，这样可以生成 d-t 曲线，每个d-t曲线下，再遍历备选速度，生成 s-t 曲线，这种方法其实只适用终点 s 自由的方式
    // 完成轨迹采样
    for (float di = 0; di <= 5; di += D_ROAD_W) {
        for (float ti = MINT; ti <= MAXT; ti += DT) {
            FrenetPath fp_without_s;
            // std::cout << "采样过程中的c_d: " << c_d << std::endl;
            QuinticPolynomial lateral_fp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);
            // 这个 for 不涉及采样，是计算已经采样得到的d-t曲线在每个时刻点状态值，这里只计算一根轨迹
            for (float _t = 0.0; _t <= ti; _t += DT) {
                fp_without_s.t.push_back(_t);
                fp_without_s.d.push_back(lateral_fp.calc_point(_t));
                fp_without_s.d_d.push_back(lateral_fp.calc_first_derivative(_t));
                fp_without_s.d_dd.push_back(lateral_fp.calc_second_derivative(_t));
                fp_without_s.d_ddd.push_back(lateral_fp.calc_third_derivative(_t));
            }
            // 当前遍历下，每一条s-t曲线复用上面刚刚生成的d-t曲线（只有一根），for循环每运行一次，生成一条完整的备选轨迹
            for (float vi = TARGET_SPEED - D_T_S * N_S_SAMPLE; vi < TARGET_SPEED + D_T_S * N_S_SAMPLE; vi += D_T_S) {
                FrenetPath fp_with_s = fp_without_s;
                QuarticPolynomial longitudinal_qp(s0, c_speed, 0.0, vi, 0.0, ti);
                for (float _t : fp_without_s.t) {
                    fp_with_s.s.push_back(longitudinal_qp.calc_point(_t));
                    fp_with_s.s_d.push_back(longitudinal_qp.calc_first_derivative(_t));
                    fp_with_s.s_dd.push_back(longitudinal_qp.calc_second_derivative(_t));
                    fp_with_s.s_ddd.push_back(longitudinal_qp.calc_third_derivative(_t));
                }
                fp_with_s.max_speed = *max_element(fp_with_s.s_d.begin(), fp_with_s.s_d.end());
                fp_with_s.max_accel = *max_element(fp_with_s.s_dd.begin(), fp_with_s.s_dd.end());

                float Jp = sum_of_power(fp_with_s.d_ddd);
                float Js = sum_of_power(fp_with_s.s_ddd);

                // 计算每条备选轨迹的代价，参考的是论文 “Optimal trajectory generation for dynamic street scenarios in a Frenét Frame“ 里面的巡航控制的代价函数计算方式
                fp_with_s.cd = KJ * Jp + KT * ti + KD * pow(fp_with_s.d.back(), 2);    // 横向代价
                // # square of diff from target speed
                float ds = pow(TARGET_SPEED - fp_with_s.s_d.back(), 2);
                fp_with_s.cv = KJ * Js + KT * ti + KD * ds;
                fp_with_s.cf = KLAT * fp_with_s.cd + KLON * fp_with_s.cv;

                fp_list.push_back(fp_with_s);
            }
        }
    }
    return fp_list;
};

// 02
// 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
void FrenetOptimalTrajectory::calc_global_paths(Vec_Path& path_list, Spline2D csp) {
    int iiii = 0;
    // 计算采样轨迹的其他参数
    // for (FrenetPath _fpp : path_list)  // 这个遍历的方式,只能遍历用，无法修改元素中的值（传的估计是值）
    for (Vec_Path::iterator _fpp = path_list.begin(); _fpp != path_list.end(); _fpp++){
        iiii++;
        // # calc global positions
        for (int i = 0; i < _fpp->s.size(); i++) {
            // 重规划局部轨迹的 s 坐标超过参考路径的最大 s 以后，超过的部分不需要计算其他参数了。
            if (_fpp->s[i] >= csp.s.back()) {
                break;
            }
            Poi_f location_cartesian = csp.calc_postion(_fpp->s[i]);
            float i_yaw_angle = csp.calc_yaw(_fpp->s[i]);
            float di = _fpp->d[i];
            float fx = location_cartesian[0] + di * cos(i_yaw_angle + M_PI / 2.0);
            float fy = location_cartesian[1] + di * sin(i_yaw_angle + M_PI / 2.0);
            _fpp->x.push_back(fx);
            _fpp->y.push_back(fy);
        }

        // # calc yaw and ds
        for (int i = 0; i < _fpp->s.size() - 1; i++) {
            // 重规划局部轨迹的 s 坐标超过参考路径的最大 s 以后，超过的部分不需要计算其他参数了。
            if (_fpp->s[i] >= csp.s.back()) {
                break;
            }
            float dx = _fpp->x[i + 1] - _fpp->x[i];
            float dy = _fpp->y[i + 1] - _fpp->y[i];
            _fpp->yaw.push_back(atan2(dy, dx));
            _fpp->ds.push_back(sqrt(pow(dx, 2) + pow(dy, 2)));
        }
        _fpp->yaw.push_back(_fpp->yaw.back());
        _fpp->ds.push_back(_fpp->ds.back());

        // # calc curvature
        for (int i = 0; i < _fpp->s.size() - 1; i++) {
            // 重规划局部轨迹的 s 坐标超过参考路径的最大 s 以后，超过的部分不需要计算其他参数了。
            if (_fpp->s[i] >= csp.s.back()) {
                break;
            }
            _fpp->c.push_back((_fpp->yaw[i + 1] - _fpp->yaw[i]) / _fpp->ds[i]);
        }
        _fpp->max_curvature = *max_element(_fpp->c.begin(), _fpp->c.end());
    }

};

bool FrenetOptimalTrajectory::check_collision(FrenetPath path, const Vec_Poi ob) {
    for (auto point : ob) {
        for (unsigned int i = 0; i < path.x.size(); i++) {
            float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
            if (dist <= ROBOT_RADIUS * ROBOT_RADIUS) {
                return false;
            }
        }
    }
    return true;
};
// 03
// 检查路径，通过限制最大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list, const Vec_Poi ob) {
    Vec_Path output_fp_list;
    //TODO: 补全代码
      for (FrenetPath fp : path_list){
       
        if (fp.max_speed <= MAX_SPEED && fp.max_accel <= MAX_ACCEL && fp.max_curvature <= MAX_CURVATURE && this->check_collision(fp, ob)){

            output_fp_list.push_back(fp);
        }
    }


    return output_fp_list;
};

// TODO: step 1 finish frenet_optimal_planning
FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd, Vec_Poi ob) {

    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
   
    calc_global_paths(fp_list, csp);

   
    Vec_Path save_paths = check_paths(fp_list, ob);

  

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for (auto path : save_paths) {
        if (min_cost >= path.cf) {
            min_cost = path.cf;
            final_path = path;
        }
    }
    return final_path;




};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(Spline2D csp, const FrenetInitialConditions& frenet_init_conditions, Vec_Poi ob) {
    float c_speed = frenet_init_conditions.c_speed;
    float c_d = frenet_init_conditions.c_d;
    float c_d_d = frenet_init_conditions.c_d_d;
    float c_d_dd = frenet_init_conditions.c_d_dd;
    float s0 = frenet_init_conditions.s0;

    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fp_list, csp);
    Vec_Path save_paths = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for (auto path : save_paths) {
        if (min_cost >= path.cf) {
            min_cost = path.cf;
            final_path = path;
        }
    }
    return final_path;
}

// }    // namespace shenlan

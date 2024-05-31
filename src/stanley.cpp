//
// Created by ender on 24-1-30.
//
#include "stanley.hpp"
#include <cmath>

//#include "control.hpp"
/**
 * 搜索目标邻近路点
 * @param vehicle_coordinates 当前车辆位置
 * @param road_point 参考轨迹（数组）
 * @return
 */

int Stanley::FindTargetIndex(const std::vector<std::vector<double>> &road_point) {
    idx = 0;
    dist.clear();
    dist_Projector_distance.clear();
    std::vector<std::ptrdiff_t>::size_type sizeOfroadpoint = road_point.size();
    auto Distance = [](double x, double y) { return sqrt(x * x + y * y); };

    for (int i = 0; i < sizeOfroadpoint - 1; i++) {
        auto OA_NORM = Distance(road_point[i][0], road_point[i][1]);
        auto AB_NORM = Distance(road_point[i + 1][0] - road_point[i][0], road_point[i + 1][1] - road_point[i][1]);

        auto Inner_product = (((road_point[i + 1][1] - road_point[i][1]) * road_point[i][0]) -
                              ((road_point[i + 1][0] - road_point[i][0]) * road_point[i][1]));

        auto Projector_distance = abs(Inner_product / AB_NORM);
        dist.emplace_back(OA_NORM);
        dist_Projector_distance.emplace_back(Projector_distance);
    }
    auto min_dist_iter = std::min_element(dist.begin(), dist.end());
    idx = static_cast<int>(std::distance(dist.begin(), min_dist_iter));;
    return idx;
}


/**
 * stanley控制
 * @param vehicle_coordinates 机器人位姿，包括x,y,yaw,v
 * @param road_point 参考轨迹的位置和参考轨迹上点的切线方向的角度 x,y,theta
 * @return 控制量
 */


double Stanley::stanleyControl(const std::vector<double> &vehicle_coordinates,
                               const std::vector<std::vector<double>> &road_point) {
    //std::cout << "fuck in 1" << std::endl;
    auto control_angle = 0.;
    std::cout << "road point size : " << road_point.size() << std::endl;
    int current_target_index = FindTargetIndex(road_point);
    std::cout << "current_target_index : " << current_target_index << std::endl;
    //cout<<current_target_index<<endl;
    //double  current_ref_point[2];

    current_ref_point.clear();
    next_point.clear();
    // 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    if (current_target_index >= road_point.size()) {
        current_target_index = road_point.size() - 1;
    }

    double e0 = road_point[current_target_index][0];
    double n0 = road_point[current_target_index][1];
    //std::cerr<<"solve start: {}";
    current_ref_point.push_back(e0);//road_point[current_target_index][i]
    current_ref_point.push_back(n0);

    double e1 = road_point[current_target_index + 1][0];
    double n1 = road_point[current_target_index + 1][1];
    next_point.push_back(e1);
    next_point.push_back(n1);


    double psi_t;
    if (current_target_index == 0) {
        psi_t = atan2(road_point[current_target_index + 1][1] - road_point[current_target_index][1],
                      road_point[current_target_index + 1][0] - road_point[current_target_index][0]);
    } else {
        psi_t = atan2(road_point[current_target_index + 1][1] - road_point[current_target_index - 1][1],
                      road_point[current_target_index + 1][0] - road_point[current_target_index - 1][0]);
    }
    //std::cout << psi_t << std::endl;
    //std::cout << "Target point Path heading without normalizeAngle: " << psi_t << std::endl;
    double PathHeading = normalizeAngle(psi_t);
    std::cout << "Target point Path heading : " << PathHeading << std::endl;
//    double VehicleHeading = vehicle_coordinates[0];
//    std::cout << "Vehicle Heading : " << vehicle_coordinates[0] << std::endl;
    double theta_e =   PathHeading;
    heading_difference = theta_e;
    std::cout << "heading_difference : " << theta_e << std::endl;

    // 计算横向误差e_y
    // 参考自https://blog.csdn.net/renyushuai900/article/details/98460758
    double e_y;
    //std::cout << "e : " << current_ref_point[0] << " n ：" << current_ref_point[1] << std::endl;
    //  (current_ref_point[1]-(current_ref_point[0]* tan(psi_t))
    std::cout << "Target point" << " ( " << current_ref_point[0] << " , " << current_ref_point[1] << " ) " << std::endl;
    //std::cout << "e1 : " << next_point[0] << " n1 ：" << next_point[1] << std::endl;
    std::cout << "next point" << " ( " << current_ref_point[0] << " , " << current_ref_point[1] << " ) " << std::endl;
    if (current_ref_point[0] * next_point[1] > next_point[0] * current_ref_point[1]) {
        e_y = -abs(current_ref_point[1]);
    } else {
        e_y = abs(current_ref_point[1]);
    }
    std::cout << "lateral error : " << dist_Projector_distance[0] << std::endl;
    //# 通过公式(5)计算转角,符号保持一致
    lateral_error = dist_Projector_distance[0];

    double v = vehicle_coordinates[1];
    std::cout << "vehicle speed : " << vehicle_coordinates[1] << std::endl;
    double V = 1;
    if (v == 0) {//std::cout << "1 " <<std::endl;
        V = 1;
    } else {//std::cout << " 2 " <<std::endl;
        V = v;
    }
    //std::cout << "V : " << V << std::endl;
    double delta_e = atan2(k * e_y, V);
    std::cout << "lateral error correction Angle : " << delta_e << std::endl;

    double delta;
    delta = normalizeAngle((delta_e + theta_e));
    if (delta > 0.5) {
        control_angle = PI / 6;
    }
    if (delta < -0.5) {
        control_angle = -PI / 6;
    }
    if (-0.5 <= delta && delta <= 0.5) {//std::cout << " 3 " <<std::endl;
        control_angle = delta;
    }
    if (-a_d_r <= delta && delta <= a_d_r && e_y <= l_e_r && e_y >= -lateral_error) {
        std::cerr << " RIGHT " << std::endl;
        control_angle = 0;
    }

    //std::cout << "calculations : " << delta << std::endl;
    std::cout << "control angle : " << control_angle << std::endl;

    return control_angle;
}
//
// Created by ender on 24-1-30.
//

#ifndef STANLEY_CONTROL_STANLEY_HPP
#define STANLEY_CONTROL_STANLEY_HPP


#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "config.hpp"

using namespace std;

#define PI 3.1415926


class Stanley {

public:
    double lateral_error;
    double heading_difference;
    std::vector<double> dist{};
    std::vector<double> current_ref_point{};
    std::vector<double> next_point{};
    std::vector<double> dist_Projector_distance{};
    double k_U = Config::lateral_error_gain_U;
    double k_D = Config::lateral_error_gain_D;
    double l_e_r = Config::lateral_error_range;
    double a_d_r = Config::angle_difference_range;
    int idx;
    //std::vector<double> vehicle_coordinates,road_point;
    int FindTargetIndex(const std::vector<std::vector<double>>& road_point);

    /**
 * 角度归一化到【-PI,PI】
 * @param angle
 * @return
 */
    static double normalizeAngle(double angle){
        while(angle > PI){
            angle -= 2*PI;
        }
        while(angle < -PI){
            angle += 2*PI;
        }
        return angle;
    };
    double stanleyControl(const std::vector<double>& vehicle_coordinates, const std::vector<std::vector<double>>& road_point);
};












#endif //STANLEY_CONTROL_STANLEY_HPP

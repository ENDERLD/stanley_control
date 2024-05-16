//
// Created by ender on 24-1-31.
//

#ifndef STANLEY_CONTROL_CONTROL_HPP
#define STANLEY_CONTROL_CONTROL_HPP


#include <chrono>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include "mw_api.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "autonomous_proto.hpp"
#include "stanley.hpp"
#include "config.hpp"

inline auto get_now = []()->uint64_t {
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
};


class Control {
public:
    std::vector<std::vector<double>> coordinates;
    std::vector<double> vehicle_coordinate;
    std::vector<double> curvature;
    std::vector<double> data;
    std::vector<double> point_lat;
    std::vector<double> point_lon;

    void Solve();
//    Extraction extraction{};
//    Solver solver{};
    Stanley stanley;
    autonomous_proto::Control control_proto;
    autonomous_proto::LocalPath lp{};
    autonomous_proto::Navigation nav{};
    autonomous_proto::VehicleState vs{};
    bool lp_parsed{false};
    bool nav_parsed{false};
    bool vs_parsed{false};
    uint64_t lp_get{0};
    uint64_t nav_get{0};
    uint64_t vs_get{0};

    std::atomic_bool solve_on{false};

    std::function<void(const autonomous_proto::Control &)> publish{};

    std::function<void(double lateral_error)> publish_2{};

    std::function<void(std::vector<double> data)> publish_3{};
//    [[nodiscard]] bool Init();
   // [[nodiscard]] bool Solve();
    void Timer();
    void LocalPath(bool parsed);
    void Navigation(bool parsed);
    void VehicleState(bool parsed);
//    Config readConfig(const std::string& filepath) {
//        Config config;
//        try {
//            pt::ptree tree;
//            pt::ini_parser::read_ini(filepath, tree);
//            config.coefficient = tree.get<double>("test.coefficient");
//            config.speed = tree.get<double>("test.speed");
//            config.allowable_error = tree.get<double>("test.allowable_error");
//            config.throttle = tree.get<double>("test.throttle");
//        } catch (std::exception const& e) {
//            std::cerr << e.what() << std::endl;
//        }
//        return config;
//    }

private:
    bool msg_got(uint64_t &now) const {
        static uint64_t lp_s = 2 * 1e9;
        static uint64_t nav_s = 2 * 1e9;
        static uint64_t vs_s = 2 * 1e9;
        return now - lp_get < lp_s && now - nav_get < nav_s && now - vs_get < vs_s;
    }

    bool msg_parsed() const {
        return lp_parsed && nav_parsed && vs_parsed;
    }
};


#endif //STANLEY_CONTROL_CONTROL_HPP

#ifndef STANLEY_CONTROL_CONFIG_HPP
#define STANLEY_CONTROL_CONFIG_HPP
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

class Config {
public:
    static double lateral_acc;
    static double lateral_error_coe;
    static double upper_speed_limit;
    static double lower_speed_limit;

    static double control_acc;
    static double control_T;
    static double lateral_error_range;
    static double angle_difference_range;
    static double steel_base;

    static std::string error_message;
    static YAML::Node stanley_control;
    static bool Load() {
        //auto ac_folder = boost::filesystem::path(getenv("HOME")) / "stanley_config";
        try {
            std::string stanly = "stanley.yaml" ;
            stanley_control = YAML::LoadFile(stanly);
            //stanley_control = YAML::LoadFile((ac_folder / "stanley.yaml").string());
            Config::lateral_acc = stanley_control["lateral_acc"].as<double>();
            Config::lateral_error_coe = stanley_control["lateral_error_coe"].as<double>();
            Config::upper_speed_limit = stanley_control["upper_speed_limit"].as<double>();
            Config::lower_speed_limit = stanley_control["lower_speed_limit"].as<double>();
            Config::control_acc = stanley_control["control_acc"].as<double>();
            Config::control_T = stanley_control["control_T"].as<double>();
            Config::lateral_error_range = stanley_control["lateral_error_range"].as<double>();
            Config::angle_difference_range = stanley_control["angle_difference_range"].as<double>();
            Config::steel_base = stanley_control["steel_base"].as<double>();
        }catch(std::exception& e) {
            error_message = "e.what(): " + std::string(e.what());
            return false;
        }
        return true;
    }

};
inline YAML::Node Config::stanley_control{};
inline std::string Config::error_message{};
inline double Config::lateral_acc{};
inline double Config::lateral_error_coe{};
inline double Config::upper_speed_limit{};
inline double Config::lower_speed_limit{};

inline double Config::control_acc{};
inline double Config::control_T{};
inline double Config::lateral_error_range{};
inline double Config::angle_difference_range{};
inline double Config::steel_base{};


#endif //STANLEY_CONTROL_CONFIG_HPP

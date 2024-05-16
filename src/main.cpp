//#include <iostream>
#include <deque>
#include <yaml-cpp/yaml.h>
#include "control.hpp"
#include "mw_api.hpp"



int main(int argc, char **argv) {
    if (not Config::Load()) {
        std::cout << "error while load config" << std::endl;
        return 0;
    }
    Control control{};
    Stanley stanley{};
    mw_init("motion_control");
    mw_master_check
    auto nh = mw_nh("motion_control");
    auto control_pub = mw_advertise(UInt8MultiArrayT, "control");
    auto control_publish = [&](const autonomous_proto::Control &control_proto) {
        static UInt8MultiArrayT msg;
        msg.data.clear();
        msg.data.resize(control_proto.ByteSizeLong());
        control_proto.SerializeToArray(msg.data.data(), msg.data.size());
        //std::cout<<"check  "<<std::endl;
        control_pub->publish(msg);
    };
    control.publish = control_publish;

    auto analyse_pub = mw_advertise(Float64T,"analyse");
    auto analyse_publish = [&](double lateral_error){
        static Float64T msg2;
        msg2.data = lateral_error;
        //std::cout<<"check : "<<lateral_error<<std::endl;
        analyse_pub->publish(msg2);
    };
    control.publish_2 = analyse_publish;

    auto data_pub = mw_advertise(Float64MultiArrayT ,"data");
    auto data_publish = [&](std::vector<double> data){
        static Float64MultiArrayT msg;
        msg.data.clear();
        msg.data.resize(data.size());
        std::copy(data.begin(), data.end(), msg.data.begin());
        data_pub ->publish(msg);
    };



    auto lp = mw_subscribe(UInt8MultiArrayT, "local_path", [&](const UInt8MultiArrayConstPtrT &msg){
        control.lp.Clear();
        control.LocalPath(control.lp.ParseFromArray(msg->data.data(), msg->data.size()));
    });
    auto nav = mw_subscribe(UInt8MultiArrayT, "navigation", [&](const UInt8MultiArrayConstPtrT &msg){
        control.nav.Clear();
        control.Navigation(control.nav.ParseFromArray(msg->data.data(), msg->data.size()));
    });
    auto vs = mw_subscribe(UInt8MultiArrayT, "vehicle_state", [&](const UInt8MultiArrayConstPtrT &msg){
        control.vs.Clear();
        control.VehicleState(control.vs.ParseFromArray(msg->data.data(), msg->data.size()));
    });
    auto timer = mw_create_timer(0.02, [&]() {
        control.Timer();

    });
    mw_spin;
    mw_shutdown;

//    spdlog::shutdown();
    return 0;
}

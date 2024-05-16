//
// Created by ender on 24-1-31.
//
#include "control.hpp"

void Control::Solve() {

    if (!vs_parsed || !nav_parsed || !lp_parsed ) {
        if (!vs_parsed) {
            std::cout << " NO VehicleState " << std::endl;
        }
        if (!nav_parsed) {
            std::cout << " NO Navigation " << std::endl;
        }
        if (!lp_parsed) {
            std::cout << " NO LocalPath " << std::endl;
        }
        std::cout << std::endl;
        return;
    }
    control_proto.Clear();
    Control::data.clear();
    //  if (not solve_on) { return true; }
    static uint64_t solve_count = 0;
    //   auto solve_start_ns = get_now();

//    spdlog::info("solve start: {}", ++solve_count);
    //std::cerr<<"timer";
    static GeographicLib::LocalCartesian proj{};
    proj.Reset(nav.position().lat().value(), nav.position().lon().value(), nav.position().alt().value());
    //std::cout<< nav.position().lat().value() << nav.position().lon().value() << nav.position().alt().value() << std::endl;
    static double e = 0, n = 0, u = 0,Cos = 0, Sin = 0 ,x = 0 , y = 0;
    double D = Config::steel_base;
    Control::coordinates.clear();
    Cos = cos(nav.orientation().yaw().value());
    Sin = sin(nav.orientation().yaw().value());

    for (const auto &point: lp.points()) {
        proj.Forward(point.lat().value(), point.lon().value(), point.alt().value(), e, n, u);
        x = Cos * e + Sin * n -D ;
        y = -Sin * e + Cos * n;
        Control::coordinates.push_back({ x , y });
    }
    //std::cout << Control::coordinates.size() << std::endl;
    double s = 0 ;
    double la = Config::lateral_acc;
    std::cout << "lateral_acc : " << la << std::endl;
    curvature.clear();
    for (int i = 0; i < Control::coordinates.size(); ++i) {
        s += sqrt(pow((coordinates[i+1][0]-coordinates[i][0]),2) + pow((coordinates[i+1][1]-coordinates[i][1]),2));
        //std::cout << "s : " << s << std::endl;
        double ds = sqrt(pow((coordinates[i+2][0]-coordinates[i+1][0]),2) + pow((coordinates[i+2][1]-coordinates[i+1][1]),2));
        //std::cout << "ds : " << ds << std::endl;

        double a0 = atan2(coordinates[i+1][1]-coordinates[i][1],coordinates[i+1][0]-coordinates[i][0]);
        double a1 = atan2(coordinates[i+2][1]-coordinates[i+1][1],coordinates[i+2][0]-coordinates[i+1][0]);
        double da = abs(a1 - a0);
        //std::cout << "da : " << da << std::endl;

        double k = da/ds;
        curvature.emplace_back(k);
        if (s >= 10) {
            break;
         }
    }
    double upper_limit_v = Config::upper_speed_limit;
    double lower_limit_v = Config::lower_speed_limit;
    double control_v = 0;
    auto max_curvature_d = std::max_element(curvature.begin(),curvature.end());
    double max_curv = *max_curvature_d;
    double cal_v = sqrt(la/max_curv);
    if (cal_v < lower_limit_v) {
        control_v = lower_limit_v;
    }
    if (cal_v > upper_limit_v) {
        control_v = upper_limit_v;
    }
    if (cal_v >= lower_limit_v &&cal_v <= upper_limit_v) {
        control_v = cal_v;
    }
    if (coordinates.size()<=2) {
        control_v = 0;
    }
    std::cout << "max_curv : " << max_curv << std::endl;
    std::cout << "control_v : " << control_v << std::endl;
    auto control_acc_ = Config::control_acc;
    double v = vs.v().Get(vs.v().size()-1);
    Control::vehicle_coordinate.clear();
    Control::vehicle_coordinate.push_back(nav.orientation().yaw().value());
    Control::vehicle_coordinate.push_back(v);//vs.driver_operation().v().value());
    //std::cerr<<"solve start: {}";
    //std::cout <<" vs.v : "<<v<< std::endl;

    double FrontWheelSteeringAngle = stanley.stanleyControl(vehicle_coordinate, coordinates);

    double tan_FrontWheelSteeringAngle = tan(FrontWheelSteeringAngle);

    double control_k = tan_FrontWheelSteeringAngle / D;
    point_lat.clear();
    point_lon.clear();
    data.emplace_back(FrontWheelSteeringAngle);
    data.emplace_back(vs.angle().Get(vs.angle().size()-1));
    data.emplace_back(FrontWheelSteeringAngle-vs.steer().Get(vs.steer().size()-1));
    data.emplace_back(control_v);
    data.emplace_back(v);
    data.emplace_back(control_v-v);
    data.emplace_back(nav.position().lat().value());
    data.emplace_back(nav.position().lon().value());
    for (const auto &point: lp.points()) {
        point_lat.emplace_back(point.alt().value());
        point_lon.emplace_back(point.lon().value());
    }
    data.emplace_back(point_lat[0]);
    data.emplace_back(point_lon[0]);

    std::cout << "control_k : " << control_k << std::endl;
    std::cout << std::endl;
    std::cout << "--------------------------------------------------------------------------------------"<<std::endl;
    std::cout << std::endl;
    control_proto.set_mode(autonomous_proto::Control_Mode_Value_autonomous);
    control_proto.add_direction(autonomous_proto::Control_Direction_Value_forward);
    control_proto.add_k(control_k);
    control_proto.add_v(control_v);
    control_proto.add_acc(control_acc_);
    publish(control_proto);
    publish_2(stanley.lateral_error);
    publish_3(data);
}


void Control::Timer() {
    Control::Solve();
//    auto now = get_now();
//
//    static uint64_t last_abnormal_input = 0;
//    auto got_msg = msg_got(now);
//    auto parsed_msg = msg_parsed();
//
//    if(msg_parsed()){
//
//   }
//    if (not got_msg || not parsed_msg) {
//        last_abnormal_input = now;
//    }
//    if (now - last_abnormal_input > 1'000'000'000) {
//        if (not solve_on) {
//           // spdlog::info("turn on solve");
//            solve_on = true;
//        }
//    } else {
//        if (solve_on) {
//            solve_on = false;
//           // spdlog::warn("turn off solve");
//        }
////        static uint64_t last_print = 0;
////        if (now - last_print >= 500'000'000) {
////            last_print = now;
////            if (not got_msg) {
//               // spdlog::warn("msg got passed: lp: {:.3f}s, nav: {:.3f}s, vs: {:.3f}s",
////                             (now - lp_get) * 1e-9,
////                             (now - nav_get) * 1e-9,
////                             (now - vs_get) * 1e-9);
////            }
////            if (not parsed_msg) {
////                spdlog::warn("msg parsed: lp: {}, nav: {}, vs: {}", lp_parsed, nav_parsed, vs_parsed);
////            }
////            if (got_msg && parsed_msg) {
////                spdlog::info("waiting input to be steady");
//            }
}


void Control::LocalPath(bool parsed) {
    lp_parsed = parsed;
    lp_get = get_now();
//    if (not Solve()){
//        return;
//   }
//    if (not Solve()) {
//        std::cout<<"solve failed"<<1;
//        //SPDLOG_ERROR("solve failed");
//    }
}

void Control::Navigation(bool parsed) {
    nav_parsed = parsed;
    nav_get = get_now();
}

void Control::VehicleState(bool parsed) {
    vs_parsed = parsed;
    vs_get = get_now();
}


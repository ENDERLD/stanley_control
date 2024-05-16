#include <iostream>
#include "mw_api.hpp"
#include "mw_bag.hpp"

int main() {
    mw_init("mw_api_test");
    mw_master_check;
    mw_nh_type nh = mw_nh("mw_api_test");

    mw_sub_type(StringT) sub = mw_subscribe(StringT, "mw_api_test", [&](const StringConstPtrT &msg) {
        std::cout << "sub callback: " << msg->data << std::endl;
    });
    mw_pub_type(StringT) pub = mw_advertise(StringT, "mw_api_test");

    StringT msg;
    uint32_t count = 0;
    mw_timer_type timer = mw_create_timer(1.0, [&](){
        msg.data = "Hello, mw api test! " + std::to_string(count++);
        pub->publish(msg);
        std::cout << "pub: " << msg.data << std::endl;
    });

    mw_spin;
    mw_shutdown;

    return 0;
}

#ifndef MW_API_MW_BAG_HPP
#define MW_API_MW_BAG_HPP

#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

struct BagMsgT {
    int64_t time{};
    std::string topic{};
    std::shared_ptr<rosbag::MessageInstance> msg_ptr{};
};

using mw_bag_reader_t = rosbag::Bag;
using mw_bag_writer_t = rosbag::Bag;

[[nodiscard]] inline auto mw_open_bag_reader(mw_bag_reader_t &reader, const boost::filesystem::path &path) {
    try {
        reader.open(path.string(), rosbag::bagmode::Read);
    } catch (const std::exception &e) {
        reader.close();
        return false;
    }
    return true;
}

[[nodiscard]] inline auto mw_open_bag_writer(mw_bag_writer_t &writer, const boost::filesystem::path &path) {
    try {
        writer.open(path.string(), rosbag::bagmode::Write);
    } catch (const std::exception &e) {
        writer.close();
        return false;
    }
    return true;
}

inline auto mw_read_bag_msgs(mw_bag_reader_t &bag, const std::vector<std::string> &topics = std::vector<std::string>{}) {
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::vector<BagMsgT> msgs{};
    BagMsgT msg{};
    for (const auto &m : view) {
        msg.msg_ptr = std::make_shared<rosbag::MessageInstance>(m);
        msg.time = (int64_t)m.getTime().toNSec();
        msg.topic = m.getTopic();
        msgs.emplace_back(msg);
    }
    return msgs;
}

template <typename MessageT>
[[nodiscard]] inline auto mw_msg_instantiate(const std::shared_ptr<rosbag::MessageInstance> &msg_ptr) {
    MessageT msg = *msg_ptr->instantiate<MessageT>();
    return msg;
}

template <typename MessageT>
inline auto mw_write_bag_msg(mw_bag_writer_t &bag, const MessageT &msg, const std::string &topic, const int64_t &time, const std::string &topic_prefix = "") {
    std::string full_topic{};
    if (topic_prefix.empty()) {
        full_topic = topic;
    } else {
        if (topic[0] == '/') {
            full_topic = topic_prefix + topic;
        } else {
            full_topic = topic_prefix + "/" + topic;
        }
    }
    ros::Time t;
    t.fromNSec(time);
    bag.write(full_topic, t, msg);
}

inline auto mw_write_bag_int64(mw_bag_writer_t &bag, const int64_t &msg, const std::string &topic, const int64_t &time, const std::string &topic_prefix = "") {
    Int64T ros_msg{};
    ros_msg.data = msg;
    mw_write_bag_msg(bag, ros_msg, topic, time, topic_prefix);
}

inline auto mw_write_bag_float64(mw_bag_writer_t &bag, const double &msg, const std::string &topic, const int64_t &time, const std::string &topic_prefix = "") {
    Float64T ros_msg{};
    ros_msg.data = msg;
    mw_write_bag_msg(bag, ros_msg, topic, time, topic_prefix);
}

inline auto mw_write_bag_int64_array(mw_bag_writer_t &bag, const std::vector<int64_t> &msg, const std::string &topic, const int64_t &time, const std::string &topic_prefix = "") {
    Int64MultiArrayT ros_msg{};
    ros_msg.data = msg;
    mw_write_bag_msg(bag, ros_msg, topic, time, topic_prefix);
}

inline auto mw_write_bag_float64_array(mw_bag_writer_t &bag, const std::vector<double> &msg, const std::string &topic, const int64_t &time, const std::string &topic_prefix = "") {
    Float64MultiArrayT ros_msg{};
    ros_msg.data = msg;
    mw_write_bag_msg(bag, ros_msg, topic, time, topic_prefix);
}

template<typename MessageT>
[[nodiscard]] inline bool mw_bag_read(const boost::filesystem::path &path,
                                      const std::vector<std::string> &topics,
                                      std::vector<MessageT> &msgs) {
    rosbag::Bag bag{};
    try {
        bag.open(path.string(), rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        bag.close();
        return false;
    }
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const auto &msg : view) {
        msgs.emplace_back(*msg.instantiate<MessageT>());
    }
    bag.close();
    return true;
}

#endif //MW_API_MW_BAG_HPP

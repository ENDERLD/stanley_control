#ifndef MW_API_MW_BAG_HPP
#define MW_API_MW_BAG_HPP

#include <boost/filesystem.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

struct BagMsgT {
    int64_t time{};
    std::string topic{};
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg_ptr{};
};

using mw_bag_reader_t = rosbag2_cpp::Reader;
using mw_bag_writer_t = rosbag2_cpp::Writer;

[[nodiscard]] inline auto mw_open_bag_reader(mw_bag_reader_t &reader, const boost::filesystem::path &path) {
    try {
        reader.open(path.string());
    } catch (const std::exception &e) {
        reader.close();
        return false;
    }
    return true;
}

[[nodiscard]] inline auto mw_open_bag_writer(mw_bag_writer_t &writer, const boost::filesystem::path &path) {
    try {
        writer.open(path.string());
    } catch (const std::exception &e) {
        writer.close();
        return false;
    }
    return true;
}

inline auto mw_read_bag_msgs(mw_bag_reader_t &bag, const std::vector<std::string> &topics = std::vector<std::string>{}) {
    bag.set_filter(rosbag2_storage::StorageFilter{topics});
    std::vector<BagMsgT> msgs{};
    BagMsgT msg{};
    while (bag.has_next()) {
        msg.msg_ptr = bag.read_next();
        msg.time = msg.msg_ptr->time_stamp;
        msg.topic = msg.msg_ptr->topic_name;
        msgs.emplace_back(msg);
    }
    bag.reset_filter();
    bag.seek(0);
    return msgs;
}

template <typename MessageT>
[[nodiscard]] inline auto mw_msg_instantiate(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &msg_ptr) {
    MessageT msg;
    rclcpp::SerializedMessage extracted_serialized_msg(*msg_ptr->serialized_data);
    rclcpp::Serialization<MessageT> serialization;
    serialization.deserialize_message(&extracted_serialized_msg, &msg);
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
    bag.write(msg, full_topic, (rclcpp::Time)time);
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
    rosbag2_cpp::Reader reader{};
    try {
        reader.open(path.string());
        reader.set_filter(rosbag2_storage::StorageFilter{topics});
    } catch (const std::exception &e) {
        reader.close();
        return false;
    }
    rclcpp::Serialization<MessageT> serialization{};
    while (reader.has_next()) {
        const rclcpp::SerializedMessage data{*reader.read_next()->serialized_data};
        msgs.emplace_back();
        serialization.deserialize_message(&data, &msgs.back());
    }
    reader.close();
    return true;
}

#endif //MW_API_MW_BAG_HPP

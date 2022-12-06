#include <iostream>
#include <boost/format.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class SummarizerBase {
public:
  virtual ~SummarizerBase() {}

  virtual void push(const unsigned char* data_ptr) = 0;
  virtual void summary() = 0;
};

template <typename T>
class Summarizer : public SummarizerBase {
public:
  Summarizer(const std::string& field_name, const std::string& datatype) : field_name(field_name), datatype(datatype) {}

  void push(const unsigned char* value_ptr) override { values.emplace_back(*reinterpret_cast<const T*>(value_ptr)); }

  void summary() override {
    const T v_first = values.front();
    const T v_last = values.back();

    std::sort(values.begin(), values.end());
    const T min = values.front();
    const T max = values.back();
    const T median = values[values.size() / 2];
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();

    if constexpr (std::is_integral_v<T>) {
      // clang-format off
      std::cout << boost::format("%-15s: datatype=%s mean=%.1f first=%d last=%d median=%d min=%d max=%d") 
                  % field_name 
                  % datatype 
                  % mean 
                  % static_cast<std::uint64_t>(v_first) 
                  % static_cast<std::uint64_t>(v_last) 
                  % static_cast<std::uint64_t>(median) 
                  % static_cast<std::uint64_t>(min) 
                  % static_cast<std::uint64_t>(max)
                << std::endl;
      // clang-format on
    } else {
      // clang-format off
      std::cout << boost::format("%-15s: datatype=%s mean=%.3f first=%.3f last=%.3f median=%.3f min=%.3f max=%.3f") 
                  % field_name 
                  % datatype 
                  % mean 
                  % v_first 
                  % v_last 
                  % median 
                  % min 
                  % max
                << std::endl;
      // clang-format on
    }
  }

private:
  const std::string field_name;
  const std::string datatype;
  std::vector<T> values;
};

class PointsInspectorNode : public rclcpp::Node {
public:
  PointsInspectorNode(rclcpp::NodeOptions& options) : rclcpp::Node("points_inspector", options) {
    RCLCPP_INFO_STREAM(this->get_logger(), "points_inspector");

    using std::placeholders::_1;
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS(), std::bind(&PointsInspectorNode::points_callback, this, _1));

    RCLCPP_INFO_STREAM(this->get_logger(), "listening to " << points_sub->get_topic_name());
  }

private:
  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    std::cout << "--- points ---" << std::endl;
    std::cout << "frame_id:" << points_msg->header.frame_id << std::endl;
    std::cout << "stamp   :" << points_msg->header.stamp.sec << " " << points_msg->header.stamp.nanosec << std::endl;
    std::cout << "size    :" << points_msg->width << " x " << points_msg->height << std::endl;

    std::vector<std::shared_ptr<SummarizerBase>> summarizers(points_msg->fields.size());
    for (size_t i = 0; i < points_msg->fields.size(); i++) {
      const auto& field = points_msg->fields[i];
      switch (field.datatype) {
        default:
          RCLCPP_WARN_STREAM(this->get_logger(), "unknown field datatype " << field.datatype);
          break;
        case sensor_msgs::msg::PointField::UINT8:
          summarizers[i] = std::make_shared<Summarizer<std::uint8_t>>(field.name, "UINT8");
          break;
        case sensor_msgs::msg::PointField::INT8:
          summarizers[i] = std::make_shared<Summarizer<std::int8_t>>(field.name, "INT8");
          break;
        case sensor_msgs::msg::PointField::UINT16:
          summarizers[i] = std::make_shared<Summarizer<std::uint16_t>>(field.name, "UINT16");
          break;
        case sensor_msgs::msg::PointField::INT16:
          summarizers[i] = std::make_shared<Summarizer<std::int16_t>>(field.name, "INT16");
          break;
        case sensor_msgs::msg::PointField::UINT32:
          summarizers[i] = std::make_shared<Summarizer<std::uint32_t>>(field.name, "UINT32");
          break;
        case sensor_msgs::msg::PointField::INT32:
          summarizers[i] = std::make_shared<Summarizer<std::int32_t>>(field.name, "INT32");
          break;
        case sensor_msgs::msg::PointField::FLOAT32:
          summarizers[i] = std::make_shared<Summarizer<float>>(field.name, "FLOAT32");
          break;
        case sensor_msgs::msg::PointField::FLOAT64:
          summarizers[i] = std::make_shared<Summarizer<double>>(field.name, "FLOAT64");
          break;
      }
    }

    for (size_t i = 0; i < points_msg->width * points_msg->height; i++) {
      const unsigned char* point_ptr = points_msg->data.data() + points_msg->point_step * i;

      for (size_t j = 0; j < points_msg->fields.size(); j++) {
        const auto& field = points_msg->fields[j];
        const unsigned char* value_ptr = point_ptr + field.offset;
        summarizers[j]->push(value_ptr);
      }
    }

    for (auto& summarizer : summarizers) {
      summarizer->summary();
    }
  }

private:
  rclcpp::SubscriptionBase::SharedPtr points_sub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<PointsInspectorNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
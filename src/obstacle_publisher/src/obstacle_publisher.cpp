#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <vector>
#include <chrono>

class ObstaclePublisher : public rclcpp::Node {
public:
  ObstaclePublisher()
  : Node("obstacle_publisher"), x_(0.0), y_(0.0), yaw_(0.0),
    last_speed_(0.0), last_steer_(0.5),
    received_speed_(false), received_steer_(false)
  {
    // Parameters
    this->declare_parameter<std::vector<double>>("positions", {});
    this->declare_parameter<std::vector<double>>("goal", {});

    // Publishers
    pose_pub_   = create_publisher<geometry_msgs::msg::PoseArray>("/obstacles", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles_marker", 10);
    goal_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
    odom_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/odom_for_dwa", 10); // <-- corrigido

    // Subscriptions
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/commands/motor/speed", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_speed_ = msg->data;
        received_speed_ = true;
      });

    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/commands/servo/position", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_steer_ = msg->data;
        received_steer_ = true;
      });

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ObstaclePublisher::onTimer, this)
    );
  }

private:
  void onTimer() {
    auto now = this->get_clock()->now();

    // Obstacles
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp    = now;
    pa.header.frame_id = "map";

    visualization_msgs::msg::MarkerArray ma;

    auto pos = this->get_parameter("positions").as_double_array();
    for (size_t i = 0; i + 1 < pos.size(); i += 2) {
      geometry_msgs::msg::Pose p;
      p.position.x = pos[i];
      p.position.y = pos[i+1];
      p.position.z = 0.0;
      pa.poses.push_back(p);

      visualization_msgs::msg::Marker m;
      m.header.stamp    = now;
      m.header.frame_id = "map";
      m.ns     = "obstacles";
      m.id     = static_cast<int>(i/2);
      m.type   = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose   = p;
      m.scale.x = m.scale.y = m.scale.z = 0.3;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
      ma.markers.push_back(m);
    }

    // Goal
    auto gp = this->get_parameter("goal").as_double_array();
    if (gp.size() == 2u) {
      geometry_msgs::msg::PoseStamped gs;
      gs.header.stamp    = now;
      gs.header.frame_id = "map";
      gs.pose.position.x = gp[0];
      gs.pose.position.y = gp[1];
      gs.pose.position.z = 0.0;
      gs.pose.orientation.w = 1.0;

      goal_pub_->publish(gs);

      visualization_msgs::msg::Marker gm;
      gm.header.stamp    = now;
      gm.header.frame_id = "map";
      gm.ns     = "goal";
      gm.id     = 0;
      gm.type   = visualization_msgs::msg::Marker::SPHERE;
      gm.action = visualization_msgs::msg::Marker::ADD;
      gm.pose   = gs.pose;
      gm.scale.x = gm.scale.y = gm.scale.z = 0.4;
      gm.color.r = 0.0; gm.color.g = 1.0; gm.color.b = 0.0; gm.color.a = 0.8;
      ma.markers.push_back(gm);
    }

    // Dead-reckoning (pose estimada)
    if (received_speed_ && received_steer_) {
      double v = last_speed_ / 23070.0 * 3.0; // m/s aproximado
      double servo_center = 0.5;
      double steer = last_steer_;
      double gamma = -(steer - servo_center) / 0.35 * (M_PI / 6.0); // rad
      double dt = 0.1; // s (100 ms)

      if (std::abs(v) > 1e-3) {
        double dx = v * std::cos(yaw_) * dt;
        double dy = v * std::sin(yaw_) * dt;
        double dyaw = (v / 0.335) * std::tan(gamma) * dt; // L ~ 0.335 m
        x_ += dx;
        y_ += dy;
        yaw_ += dyaw;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now;
      pose.header.frame_id = "odom";
      pose.pose.position.x = x_;
      pose.pose.position.y = y_;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw_/2.0);
      pose.pose.orientation.w = std::cos(yaw_/2.0);
      odom_pub_->publish(pose);
    }

    // Publicações
    pose_pub_->publish(pa);
    marker_pub_->publish(ma);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, yaw_;
  double last_speed_, last_steer_;
  bool received_speed_, received_steer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}

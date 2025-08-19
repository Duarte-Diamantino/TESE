#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class SplineVisualizer : public rclcpp::Node {
public:
  SplineVisualizer()
  : Node("spline_visualizer")
  {
    // Parâmetros para a spline y = a x^2 + b x + c
    declare_parameter<double>("a", 1.0);
    declare_parameter<double>("b", 0.0);
    declare_parameter<double>("c", 0.0);
    declare_parameter<double>("x_min", 0.0);
    declare_parameter<double>("x_max", 5.0);

    // Publisher para visualização (Marker)
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);
    // Publisher para o Path (MPC)
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "reference_path", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SplineVisualizer::publishSplineAndPath, this));
  }

private:
  void publishSplineAndPath() {
    // Lê parâmetros
    double a, b, c, x_min, x_max;
    get_parameter("a", a);
    get_parameter("b", b);
    get_parameter("c", c);
    get_parameter("x_min", x_min);
    get_parameter("x_max", x_max);

    // Prepara o Marker LINE_STRIP
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = now();
    m.ns = "quadratic_spline";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.g = 1.0;
    m.color.a = 1.0;

    // Prepara o Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = m.header.stamp;

    const int N = 100;
    double dx = (x_max - x_min) / N;

    m.points.reserve(N+1);
    path_msg.poses.reserve(N+1);

    for (int i = 0; i <= N; ++i) {
      double x = x_min + i * dx;
      double y = a*x*x + b*x + c;

      // adiciona ponto ao Marker
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = 0.0;
      m.points.push_back(p);

      // adiciona PoseStamped ao Path
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position = p;
      ps.pose.orientation.w = 1.0;  // sem rotação
      path_msg.poses.push_back(ps);
    }

    // publica ambos
    marker_pub_->publish(m);
    path_pub_->publish(path_msg);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           path_pub_;
  rclcpp::TimerBase::SharedPtr                               timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SplineVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>

struct Pt { double x, y; };

class DwaSplineFollower : public rclcpp::Node {
public:
  DwaSplineFollower()
  : Node("dwa_spline_follower"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    // Initialize state variables
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    have_odom_ = false;
    have_path_ = false;
    joy_override_ = false;
    last_gamma_ = 0.0;

    // weights
    declare_parameter("w_path", 2.0);
    declare_parameter("w_vel", 0.5);
    get_parameter("w_path", w_path_);
    get_parameter("w_vel",  w_vel_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&DwaSplineFollower::odomCb, this, std::placeholders::_1)
    );
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/middle_lane_path", 10,
      std::bind(&DwaSplineFollower::pathCb, this, std::placeholders::_1)
    );
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&DwaSplineFollower::joyCb, this, std::placeholders::_1)
    );

    speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/commands/motor/speed", 10
    );
    steer_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/commands/servo/position", 10
    );
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predicted_trajectories", 10
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(DT * 1000)),
      std::bind(&DwaSplineFollower::update, this)
    );
  }

private:
  // state
  double x_;
  double y_;
  double yaw_;
  bool have_odom_;
  bool have_path_;
  bool joy_override_;
  std::vector<Pt> path_pts_;
  double last_gamma_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // parameters
  double w_path_;
  double w_vel_;

  // constants
  static constexpr double INITIAL_HORIZON = 2.0;
  static constexpr double DT = 0.1;
  static constexpr int NV = 15, NG = 15;
  static constexpr double V_MAX = 1.5;
  static constexpr double GAMMA_MAX = M_PI/6.0;
  static constexpr double WHEEL_BASE = 0.335;
  static constexpr double MAX_RAW = 10070.0;
  static constexpr double SERVO_C = 0.5304;
  static constexpr double SERVO_MAX = 0.94299;
  static constexpr double SERVO_MIN = 0.11781;

  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    auto &q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z));
    have_odom_ = true;
  }

  void pathCb(const nav_msgs::msg::Path::SharedPtr msg) {
      try {
          // Use the most recent available transform instead of exact timestamp
          auto transform = tf_buffer_->lookupTransform(
              "odom", msg->header.frame_id, tf2::TimePointZero);
          
          path_pts_.clear();
          for (auto &p : msg->poses) {
              geometry_msgs::msg::PoseStamped pose_out;
              tf2::doTransform(p, pose_out, transform);
              path_pts_.push_back({pose_out.pose.position.x, pose_out.pose.position.y});
          }
          have_path_ = true;
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
      }
  }

  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_override_ = std::any_of(
      msg->buttons.begin(), msg->buttons.end(),
      [](int b){ return b>0; }
    );
  }

  std::pair<Pt,Pt> getTargetPoints() const {
    size_t n = path_pts_.size();
    return { path_pts_[n/2], path_pts_[n/8] };
  }

  double computeCost(const std::vector<Pt> &traj,
                   const Pt &mid, const Pt &end,
                   double v) const {
    double dmid=1e9, dend=1e9;
    for (auto &pt: traj) {
      dmid = std::min(dmid, std::hypot(pt.x-mid.x, pt.y-mid.y));
      dend = std::min(dend, std::hypot(pt.x-end.x, pt.y-end.y));
    }
    return -w_path_*(dmid+dend) - w_vel_*std::abs(v);
  }

  void update() {
    if (!have_odom_ || !have_path_ || joy_override_) return;

    auto targets = getTargetPoints();
    Pt mid = targets.first, end = targets.second;

    visualization_msgs::msg::MarkerArray ma;
    auto now = this->now();
    int id=0;

    // yellow spheres on mid/end of spline
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id="odom"; m.header.stamp=now;
      m.ns="targets"; m.id=id++; m.type=m.SPHERE;
      m.action=m.ADD; m.scale.x=0.1; m.scale.y=0.1; m.scale.z=0.1;
      m.color.r=1.0; m.color.g=1.0; m.color.b=0.0; m.color.a=1.0;
      m.pose.position.x=mid.x; m.pose.position.y=mid.y; m.pose.position.z=0;
      ma.markers.push_back(m);
      m.id=id++;
      m.pose.position.x=end.x; m.pose.position.y=end.y;
      ma.markers.push_back(m);
    }

    double best_cost=-1e9, best_v=0, best_g=0;
    std::vector<Pt> best_traj;

    double dv = 2*V_MAX/(NV-1);
    double dg = 2*GAMMA_MAX/(NG-1);

    for(int iv=0; iv<NV; ++iv) {
      double v = -V_MAX + iv*dv;
      for(int ig=0; ig<NG; ++ig) {
        double g = -GAMMA_MAX + ig*dg;

        // simulate
        double sx=x_, sy=y_, syaw=yaw_;
        std::vector<Pt> traj;
        traj.reserve(int(INITIAL_HORIZON/DT)+1);
        for(int k=0; k<=int(INITIAL_HORIZON/DT); ++k) {
          traj.push_back({sx, sy});
          sx   += v*std::cos(syaw)*DT;
          sy   += v*std::sin(syaw)*DT;
          syaw += (v/WHEEL_BASE)*std::tan((v>=0?g:-g))*DT;
        }

        double cost = computeCost(traj, mid, end, v);

        // gray trajectories
        visualization_msgs::msg::Marker m;
        m.header.frame_id="odom"; m.header.stamp=now;
        m.ns="all_traj"; m.id=id++; m.type=m.LINE_STRIP;
        m.action=m.ADD; m.scale.x=0.01;
        m.color.r=0.5; m.color.g=0.5; m.color.b=0.5; m.color.a=0.6;
        for(auto &p:traj) {
          geometry_msgs::msg::Point P; P.x=p.x; P.y=p.y; P.z=0;
          m.points.push_back(P);
        }
        ma.markers.push_back(m);

        if(cost>best_cost) {
          best_cost=cost; best_v=v; best_g=g; best_traj=traj;
        }
      }
    }

    last_gamma_=best_g;

    // turquoise best
    {
      visualization_msgs::msg::Marker mb;
      mb.header.frame_id="odom"; mb.header.stamp=now;
      mb.ns="best_traj"; mb.id=id++; mb.type=mb.LINE_STRIP;
      mb.action=mb.ADD; mb.scale.x=0.02;
      mb.color.r=0; mb.color.g=1; mb.color.b=1; mb.color.a=1;
      for(auto &p:best_traj) {
        geometry_msgs::msg::Point P; P.x=p.x; P.y=p.y; P.z=0;
        mb.points.push_back(P);
      }
      ma.markers.push_back(mb);
    }

    marker_pub_->publish(ma);

    // publish commands
    std_msgs::msg::Float64 sp, st;
    sp.data = std::clamp(best_v/V_MAX*MAX_RAW, -MAX_RAW, MAX_RAW);
    speed_pub_->publish(sp);

    double span = SERVO_MAX - SERVO_C;
    double corr = (best_v>=0?-best_g:best_g);
    st.data = std::clamp(SERVO_C + (corr/GAMMA_MAX)*span,
                         SERVO_MIN, SERVO_MAX);
    steer_pub_->publish(st);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaSplineFollower>());
  rclcpp::shutdown();
  return 0;
}
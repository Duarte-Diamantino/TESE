#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <functional>
#include <mutex>
#include <memory>
#include <cstdint>

struct Pt { double x, y; };

class DwaGlobalFollower : public rclcpp::Node {
public:
  DwaGlobalFollower();

private:
  // State variables
  double x_, y_, yaw_;
  bool have_odom_;
  bool have_path_;
  bool joy_override_;
  double last_gamma_;
  double last_v_;
  int last_direction_; // 1 for forward, -1 for backward
  std::vector<Pt> path_pts_;
  std::vector<Pt> obstacles_;
  mutable std::mutex obstacles_mutex_; // Keep access thread-safe in const methods
  double min_safe_distance_;
  double lookahead_dist_;

  // Parameters
  double w_path_;
  double w_vel_;
  double w_goal_;
  double w_obs_;
  double w_gamma_;
  double w_index_;     // prioritize smaller path indices
  double w_pose_snap_; // prefer trajectories that snap onto path fast (incl. k=0)
  double horizon_s_;
  int window_pts_;
  int first_level_samples_;
  int second_level_samples_;

  // Point-hit constraint parameters
  bool require_two_hits_;
  int required_hits_;      // how many consecutive path points must be hit (default 2)
  double path_hit_radius_; // distance threshold to consider a path point "hit"
  bool relax_if_none_;     // if no candidate satisfies the hit constraint, relax it

  // Pose-on-path preference
  bool prefer_pose_on_path_;
  double pose_hit_radius_;

  // Constants
  static constexpr double DT = 0.1;
  static constexpr double V_MAX = 3.0;
  static constexpr double GAMMA_MAX = M_PI/6.0;
  static constexpr double WHEEL_BASE = 0.335;
  static constexpr double MAX_RAW = 23070.0;
  static constexpr double SERVO_C = 0.5304;
  static constexpr double SERVO_MAX = 0.94299;
  static constexpr double SERVO_MIN = 0.11781;
  static constexpr double GOAL_TOL = 0.05;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_pub_; // gradient markers for path
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCb(const nav_msgs::msg::Path::SharedPtr msg);
  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  
  // Helper functions
  size_t findClosestIdx() const;
  size_t closestPathIdxToPoint(const Pt& p) const;
  std::vector<Pt> simulateTrajectory(double v, double g, int steps) const;
  bool isTrajectorySafe(const std::vector<Pt>& traj) const;
  std::pair<double, double> getLocalGoal() const;
  bool coversNextNPointsOrdered(const std::vector<Pt>& traj, size_t start_idx, int N, double radius) const; // Constraint
  double minDistanceToPath(const Pt& p) const; // distance from a point to nearest path sample
  int earliestPathHit(const std::vector<Pt>& traj, double radius) const; // first time we touch path
  double computeCost(const std::vector<Pt>& traj, const std::pair<double, double>& local_goal, double v, double g) const;
  void publishPathMarkers() const;
  void update();
};

DwaGlobalFollower::DwaGlobalFollower()
: Node("dwa_global_follower"),
  x_(0), y_(0), yaw_(0),
  have_odom_(false), have_path_(false),
  joy_override_(false), last_gamma_(0),
  last_v_(0), last_direction_(1),
  min_safe_distance_(0.5), // safety radius around obstacles
  lookahead_dist_(2.0),    // local-goal distance along path
  w_path_(2.0), w_vel_(0.5), w_goal_(2.0), w_obs_(1.0), w_gamma_(0.5),
  w_index_(3.0),           // default weight favoring smaller indices
  w_pose_snap_(3.0),       // prefer snapping onto path quickly
  horizon_s_(1.0),
  window_pts_(50),
  first_level_samples_(6),
  second_level_samples_(7),
  require_two_hits_(true),
  required_hits_(2),
  path_hit_radius_(0.18),
  relax_if_none_(true),
  prefer_pose_on_path_(true),
  pose_hit_radius_(0.18)
{
  // Declare parameters
  declare_parameter("w_path", w_path_);
  declare_parameter("w_vel", w_vel_);
  declare_parameter("w_goal", w_goal_);
  declare_parameter("w_obs", w_obs_);
  declare_parameter("w_gamma", w_gamma_);
  declare_parameter("w_index", w_index_);
  declare_parameter("w_pose_snap", w_pose_snap_);
  declare_parameter("horizon_s", horizon_s_);
  declare_parameter("lookahead_dist", lookahead_dist_);
  declare_parameter("window_pts", window_pts_);
  declare_parameter("first_level_samples", first_level_samples_);
  declare_parameter("second_level_samples", second_level_samples_);
  declare_parameter("min_safe_distance", min_safe_distance_);
  declare_parameter("require_two_hits", require_two_hits_);
  declare_parameter("required_hits", required_hits_);
  declare_parameter("path_hit_radius", path_hit_radius_);
  declare_parameter("relax_if_none", relax_if_none_);
  declare_parameter("prefer_pose_on_path", prefer_pose_on_path_);
  declare_parameter("pose_hit_radius", pose_hit_radius_);

  // Get parameters
  get_parameter("w_path", w_path_);
  get_parameter("w_vel", w_vel_);
  get_parameter("w_goal", w_goal_);
  get_parameter("w_obs", w_obs_);
  get_parameter("w_gamma", w_gamma_);
  get_parameter("w_index", w_index_);
  get_parameter("w_pose_snap", w_pose_snap_);
  get_parameter("horizon_s", horizon_s_);
  get_parameter("lookahead_dist", lookahead_dist_);
  get_parameter("window_pts", window_pts_);
  get_parameter("first_level_samples", first_level_samples_);
  get_parameter("second_level_samples", second_level_samples_);
  get_parameter("min_safe_distance", min_safe_distance_);
  get_parameter("require_two_hits", require_two_hits_);
  get_parameter("required_hits", required_hits_);
  get_parameter("path_hit_radius", path_hit_radius_);
  get_parameter("relax_if_none", relax_if_none_);
  get_parameter("prefer_pose_on_path", prefer_pose_on_path_);
  get_parameter("pose_hit_radius", pose_hit_radius_);

  // Subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&DwaGlobalFollower::odomCb, this, std::placeholders::_1));
  
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/reference_path", 10,
    std::bind(&DwaGlobalFollower::pathCb, this, std::placeholders::_1));
  
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&DwaGlobalFollower::joyCb, this, std::placeholders::_1));
  
  obstacles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/obstacles_marker", 10,
    std::bind(&DwaGlobalFollower::obstaclesCb, this, std::placeholders::_1));

  // Publishers
  speed_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
  steer_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_trajectories", 10);
  path_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/reference_path_markers", 10);

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(DT * 1000)),
    std::bind(&DwaGlobalFollower::update, this));
}

void DwaGlobalFollower::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  auto& q = msg->pose.pose.orientation;
  yaw_ = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  have_odom_ = true;
}

void DwaGlobalFollower::pathCb(const nav_msgs::msg::Path::SharedPtr msg) {
  // Reorder the received path by an inferred "ordem": prefer header.seq, then timestamp, then fallback to arrival index
  struct OrderedPt { Pt pt; long long ordem; };
  std::vector<OrderedPt> ordered;
  ordered.reserve(msg->poses.size());

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto &ps = msg->poses[i];
    long long ordem = 0;
    /* ROS 2 Humble: std_msgs/Header has no 'seq'. Use stamp or arrival index instead. */
    if (ordem == 0) {
      long long stamp = static_cast<long long>(ps.header.stamp.sec) * 1000000000LL + ps.header.stamp.nanosec;
      if (stamp > 0) ordem = stamp;
    }
    if (ordem == 0) {
      ordem = static_cast<long long>(i);
    }
    ordered.push_back({{ps.pose.position.x, ps.pose.position.y}, ordem});
  }

  std::sort(ordered.begin(), ordered.end(), [](const OrderedPt &a, const OrderedPt &b){
    return a.ordem < b.ordem;
  });

  path_pts_.clear();
  path_pts_.reserve(ordered.size());
  for (const auto &op : ordered) path_pts_.push_back(op.pt);
  have_path_ = !path_pts_.empty();

  // Publish gradient markers along the ordered path to visualize index/order
  publishPathMarkers();
}

void DwaGlobalFollower::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy_override_ = std::any_of(
    msg->buttons.begin(), msg->buttons.end(), [](int b){return b>0;});
}

void DwaGlobalFollower::obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
  
  for (const auto& marker : msg->markers) {
    if (marker.ns == "obstacle" || marker.ns == "obstacles") {
      obstacles_.push_back({marker.pose.position.x, marker.pose.position.y});
    }
  }
}

size_t DwaGlobalFollower::findClosestIdx() const {
  if (path_pts_.empty()) return 0;
  
  size_t idx = 0;
  double md = 1e9;
  for(size_t i = 0; i < path_pts_.size(); ++i) {
    double d = std::hypot(path_pts_[i].x - x_, path_pts_[i].y - y_);
    if(d < md) { md = d; idx = i; }
  }
  return idx;
}

size_t DwaGlobalFollower::closestPathIdxToPoint(const Pt& p) const {
  if (path_pts_.empty()) return 0;
  size_t idx = 0;
  double md = 1e9;
  for (size_t i = 0; i < path_pts_.size(); ++i) {
    double d = std::hypot(path_pts_[i].x - p.x, path_pts_[i].y - p.y);
    if (d < md) { md = d; idx = i; }
  }
  return idx;
}

std::vector<Pt> DwaGlobalFollower::simulateTrajectory(double v, double g, int steps) const {
  std::vector<Pt> traj;
  traj.reserve(steps+1);
  double sx = x_, sy = y_, syaw = yaw_;
  for(int k = 0; k <= steps; ++k) {
    traj.push_back({sx, sy});
    sx += v * std::cos(syaw) * DT;
    sy += v * std::sin(syaw) * DT;
    double gamma_effective = (v >= 0) ? g : -g;
    syaw += (v / WHEEL_BASE) * std::tan(gamma_effective) * DT;
  }
  return traj;
}

bool DwaGlobalFollower::isTrajectorySafe(const std::vector<Pt>& traj) const {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  
  for (const auto& traj_pt : traj) {
    for (const auto& obs : obstacles_) {
      double dx = traj_pt.x - obs.x;
      double dy = traj_pt.y - obs.y;
      double distance = std::hypot(dx, dy);
      
      if (distance < min_safe_distance_) {
        return false;
      }
    }
  }
  return true;
}

std::pair<double, double> DwaGlobalFollower::getLocalGoal() const {
  if (path_pts_.empty()) {
    return {x_, y_};
  }
  
  size_t closest_idx = findClosestIdx();
  double accumulated_dist = 0.0;
  
  for (size_t i = closest_idx; i < path_pts_.size() - 1; i++) {
    double dx = path_pts_[i+1].x - path_pts_[i].x;
    double dy = path_pts_[i+1].y - path_pts_[i].y;
    double seg_dist = std::hypot(dx, dy);
    
    if (accumulated_dist + seg_dist >= lookahead_dist_) {
      double ratio = (lookahead_dist_ - accumulated_dist) / seg_dist;
      double goal_x = path_pts_[i].x + ratio * dx;
      double goal_y = path_pts_[i].y + ratio * dy;
      return {goal_x, goal_y};
    }
    
    accumulated_dist += seg_dist;
  }
  
  return {path_pts_.back().x, path_pts_.back().y};
}

bool DwaGlobalFollower::coversNextNPointsOrdered(const std::vector<Pt>& traj, size_t start_idx, int N, double radius) const {
  if (path_pts_.empty() || N <= 0) return true;
  size_t last_traj_hit = 0;
  size_t end_idx = std::min(path_pts_.size(), start_idx + static_cast<size_t>(N));
  for (size_t j = start_idx; j < end_idx; ++j) {
    bool hit = false;
    for (size_t ti = last_traj_hit; ti < traj.size(); ++ti) {
      if (std::hypot(traj[ti].x - path_pts_[j].x, traj[ti].y - path_pts_[j].y) <= radius) {
        last_traj_hit = ti + 1; // enforce order along trajectory
        hit = true;
        break;
      }
    }
    if (!hit) return false; // failed to hit this path point in order
  }
  return true;
}

// Helpers for pose-on-path snap preference

double DwaGlobalFollower::minDistanceToPath(const Pt& p) const {
  double md = 1e9;
  for (const auto& q : path_pts_) {
    double d = std::hypot(p.x - q.x, p.y - q.y);
    if (d < md) md = d;
  }
  return md;
}

int DwaGlobalFollower::earliestPathHit(const std::vector<Pt>& traj, double radius) const {
  for (size_t k = 0; k < traj.size(); ++k) {
    if (minDistanceToPath(traj[k]) <= radius) return static_cast<int>(k);
  }
  return -1; // never touched within radius
}

void DwaGlobalFollower::publishPathMarkers() const {
  if (!have_path_ || path_pts_.empty()) return;

  visualization_msgs::msg::MarkerArray ma;
  auto now = this->now();

  for (size_t i = 0; i < path_pts_.size(); ++i) {
    double t = (path_pts_.size() > 1) ? (static_cast<double>(i) / (path_pts_.size() - 1)) : 0.0; // 0..1
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "reference_path_points";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = path_pts_[i].x;
    m.pose.position.y = path_pts_[i].y;
    m.pose.position.z = 0.0;
    m.scale.x = 0.07; // small dots on top of the path
    m.scale.y = 0.07;
    m.scale.z = 0.07;
    // Blue (low index) -> Red (high index)
    m.color.r = t;
    m.color.g = 0.0;
    m.color.b = 1.0 - t;
    m.color.a = 0.9;
    ma.markers.push_back(m);
  }

  path_markers_pub_->publish(ma);
}

double DwaGlobalFollower::computeCost(const std::vector<Pt>& traj, 
                                     const std::pair<double, double>& local_goal, 
                                     double v, double g) const {
  double cost = 0.0;
  
  // 1. Path following cost (distance to path points)
  if (!path_pts_.empty()) {
    double sumd = 0;
    size_t N = path_pts_.size();
    size_t start = findClosestIdx();
    for(int k = 0; k < window_pts_; ++k) {
      const Pt &t = path_pts_[(start + k) % N];
      double dmin = 1e9;
      for(const auto &p : traj) {
        dmin = std::min(dmin, std::hypot(p.x - t.x, p.y - t.y));
      }
      sumd += dmin;
    }
    cost += -w_path_ * sumd;
  }
  
  // 2. Goal distance cost
  double dx_end = traj.back().x - local_goal.first;
  double dy_end = traj.back().y - local_goal.second;
  double dist_to_goal = std::hypot(dx_end, dy_end);
  cost += -w_goal_ * dist_to_goal;
  
  // 3. Obstacle clearance cost (maximize min distance)
  double min_obstacle_dist = 1e9;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for (const auto& pt : traj) {
      for (const auto& obs : obstacles_) {
        double d = std::hypot(pt.x - obs.x, pt.y - obs.y);
        if (d < min_obstacle_dist) min_obstacle_dist = d;
      }
    }
  }
  cost += w_obs_ * min_obstacle_dist;
  
  // 4. Velocity cost (prefer lower |v| to be smoother)
  cost += -w_vel_ * std::abs(v);
  
  // 5. Steering change penalty (prefer not to jerk)
  cost -= w_gamma_ * std::abs(g - last_gamma_);
  
  // 6. Direction change penalty
  if ((v >= 0) != (last_v_ >= 0)) {
    cost -= 1000.0;
  }

  // 7. Index-bias cost — favor trajectories that end near LOWER path indices
  if (!path_pts_.empty()) {
    size_t idx_end = closestPathIdxToPoint(traj.back());
    double denom = std::max<size_t>(1, path_pts_.size() - 1);
    double idx_score = 1.0 - static_cast<double>(idx_end) / static_cast<double>(denom);
    cost += w_index_ * idx_score;
  }

  // 8. Pose-on-path snap reward — reward trajectories that touch the path early (k=0 best)
  if (!path_pts_.empty() && prefer_pose_on_path_) {
    int k = earliestPathHit(traj, pose_hit_radius_);
    if (k >= 0) {
      int steps = std::max(1, (int)std::round(horizon_s_ / DT));
      double score = 1.0 - static_cast<double>(k) / static_cast<double>(steps); // 1 at k=0 .. 0 at end
      cost += w_pose_snap_ * score;
    }
  }
  
  return cost;
}

void DwaGlobalFollower::update() {
  if(!have_odom_ || !have_path_ || joy_override_ || path_pts_.empty()) {
    return;
  }

  // Check if reached final goal
  double dist_to_goal = std::hypot(x_ - path_pts_.back().x, y_ - path_pts_.back().y);
  if (dist_to_goal < GOAL_TOL) {
    std_msgs::msg::Float64 stop_speed;
    stop_speed.data = 0.0;
    speed_pub_->publish(stop_speed);
    marker_pub_->publish(visualization_msgs::msg::MarkerArray());
    return;
  }

  // Get local goal
  auto local_goal = getLocalGoal();
  size_t start_idx = findClosestIdx();

  // Prepare visualization
  visualization_msgs::msg::MarkerArray ma;
  auto now = this->now();
  int id = 0;
  int steps = static_cast<int>(horizon_s_ / DT);

  // 1. Add green ball at robot position
  visualization_msgs::msg::Marker robot_marker;
  robot_marker.header.frame_id = "map";
  robot_marker.header.stamp = now;
  robot_marker.ns = "robot_position";
  robot_marker.id = id++;
  robot_marker.type = visualization_msgs::msg::Marker::SPHERE;
  robot_marker.action = visualization_msgs::msg::Marker::ADD;
  robot_marker.pose.position.x = x_;
  robot_marker.pose.position.y = y_;
  robot_marker.pose.position.z = 0.0;
  robot_marker.scale.x = 0.3;
  robot_marker.scale.y = 0.3;
  robot_marker.scale.z = 0.3;
  robot_marker.color.r = 0.0;
  robot_marker.color.g = 1.0;
  robot_marker.color.b = 0.0;
  robot_marker.color.a = 1.0;
  ma.markers.push_back(robot_marker);

  // 2. First level: main directions
  std::vector<std::tuple<double, double, double, std::vector<Pt>>> first_level;
  bool any_with_hits_first = false;
  
  // Main directions (forward and backward)
  std::vector<std::pair<double, double>> directions = {
    {V_MAX, 0.0},          // Forward straight
    {V_MAX, GAMMA_MAX/2},  // Forward left
    {V_MAX, -GAMMA_MAX/2}, // Forward right
    {-V_MAX, 0.0},         // Backward straight
    {-V_MAX, GAMMA_MAX/2}, // Backward left
    {-V_MAX, -GAMMA_MAX/2} // Backward right
  };

  for(int i = 0; i < first_level_samples_; ++i) {
    double v = directions[i].first;
    double g = directions[i].second;
    auto traj = simulateTrajectory(v, g, steps);
    
    // Check safety
    if (!isTrajectorySafe(traj)) {
      // Visualize unsafe trajectories in red
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "unsafe_traj";
      m.id = id++;
      m.type = m.LINE_STRIP;
      m.action = m.ADD;
      m.scale.x = 0.02;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 0.6;
      for(auto &p : traj) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        m.points.push_back(point);
      }
      ma.markers.push_back(m);
      continue;
    }

    // Check ordered 2-point coverage
    bool cover_ok = !require_two_hits_ || coversNextNPointsOrdered(traj, start_idx, required_hits_, path_hit_radius_);
    if (!cover_ok) {
      // Visualize skipped-coverage trajectories in orange
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "skip_traj";
      m.id = id++;
      m.type = m.LINE_STRIP;
      m.action = m.ADD;
      m.scale.x = 0.02;
      m.color.r = 1.0;
      m.color.g = 0.5;
      m.color.b = 0.0;
      m.color.a = 0.6;
      for(auto &p : traj) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        m.points.push_back(point);
      }
      ma.markers.push_back(m);
    } else {
      any_with_hits_first = true;
      double cost = computeCost(traj, local_goal, v, g);
      first_level.emplace_back(cost, v, g, traj);
    }
  }

  // If none satisfied the coverage, we allow all safe ones
  if (!any_with_hits_first && relax_if_none_) {
    first_level.clear();
    for(int i = 0; i < first_level_samples_; ++i) {
      double v = directions[i].first;
      double g = directions[i].second;
      auto traj = simulateTrajectory(v, g, steps);
      if (!isTrajectorySafe(traj)) continue;
      double cost = computeCost(traj, local_goal, v, g);
      first_level.emplace_back(cost, v, g, traj);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Relaxing two-hit constraint at first level (blocked segment?)");
  }

  // Sort by cost (best first)
  std::sort(first_level.begin(), first_level.end(), 
    [](const auto& a, const auto& b){ return std::get<0>(a) > std::get<0>(b); });

  // Visualize best first-level trajectories (yellow)
  for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
    const auto& traj = std::get<3>(first_level[i]);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "main_traj";
    m.id = id++;
    m.type = m.LINE_STRIP;
    m.action = m.ADD;
    m.scale.x = 0.03;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    for(auto &p : traj) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0;
      m.points.push_back(point);
    }
    ma.markers.push_back(m);
  }

  // 3. Second level: refinement around best directions
  double best_cost = -1e9;
  double best_v = 0;
  double best_g = 0;
  std::vector<Pt> best_traj;
  bool any_with_hits_second = false;

  for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
    double base_v = std::get<1>(first_level[i]);
    double base_g = std::get<2>(first_level[i]);
    
    double v_range = (base_v > 0) ? V_MAX/2 : V_MAX/3;
    double g_range = GAMMA_MAX/2;
    
    double dv = 2 * v_range / (second_level_samples_ - 1);
    double dg = 2 * g_range / (second_level_samples_ - 1);
    
    for(int iv = 0; iv < second_level_samples_; ++iv) {
      double v = std::clamp(base_v - v_range + iv * dv, -V_MAX, V_MAX);
      for(int ig = 0; ig < second_level_samples_; ++ig) {
        double g = std::clamp(base_g - g_range + ig * dg, -GAMMA_MAX, GAMMA_MAX);
        
        auto traj = simulateTrajectory(v, g, steps);
        
        // Skip unsafe trajectories
        if (!isTrajectorySafe(traj)) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "map";
          m.header.stamp = now;
          m.ns = "unsafe_traj";
          m.id = id++;
          m.type = m.LINE_STRIP;
          m.action = m.ADD;
          m.scale.x = 0.015;
          m.color.r = 1.0;
          m.color.g = 0.0;
          m.color.b = 0.0;
          m.color.a = 0.4;
          for(auto &p : traj) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            m.points.push_back(point);
          }
          ma.markers.push_back(m);
          continue;
        }

        // Check ordered two-hit coverage
        bool cover_ok = !require_two_hits_ || coversNextNPointsOrdered(traj, start_idx, required_hits_, path_hit_radius_);
        if (!cover_ok) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "map";
          m.header.stamp = now;
          m.ns = "skip_traj";
          m.id = id++;
          m.type = m.LINE_STRIP;
          m.action = m.ADD;
          m.scale.x = 0.015;
          m.color.r = 1.0;
          m.color.g = 0.5;
          m.color.b = 0.0;
          m.color.a = 0.4;
          for(auto &p : traj) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            m.points.push_back(point);
          }
          ma.markers.push_back(m);
          continue;
        }
        any_with_hits_second = true;
        
        double cost = computeCost(traj, local_goal, v, g);
        
        // Visualize safe trajectories (green)
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now;
        m.ns = "second_level";
        m.id = id++;
        m.type = m.LINE_STRIP;
        m.action = m.ADD;
        m.scale.x = 0.015;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 0.4;
        for(auto &p : traj) {
          geometry_msgs::msg::Point point;
          point.x = p.x;
          point.y = p.y;
          point.z = 0;
          m.points.push_back(point);
        }
        ma.markers.push_back(m);
        
        if(cost > best_cost) {
          best_cost = cost;
          best_v = v;
          best_g = g;
          best_traj = traj;
        }
      }
    }
  }

  // Relaxation fallback if nothing satisfied the hit constraint
  if (!any_with_hits_second && relax_if_none_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Relaxing two-hit constraint at second level (blocked segment?)");
    best_cost = -1e9;
    best_traj.clear();
    for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
      double base_v = std::get<1>(first_level[i]);
      double base_g = std::get<2>(first_level[i]);
      double v_range = (base_v > 0) ? V_MAX/2 : V_MAX/3;
      double g_range = GAMMA_MAX/2;
      double dv = 2 * v_range / (second_level_samples_ - 1);
      double dg = 2 * g_range / (second_level_samples_ - 1);
      for(int iv = 0; iv < second_level_samples_; ++iv) {
        double v = std::clamp(base_v - v_range + iv * dv, -V_MAX, V_MAX);
        for(int ig = 0; ig < second_level_samples_; ++ig) {
          double g = std::clamp(base_g - g_range + ig * dg, -GAMMA_MAX, GAMMA_MAX);
          auto traj = simulateTrajectory(v, g, steps);
          if (!isTrajectorySafe(traj)) continue;
          double cost = computeCost(traj, local_goal, v, g);
          if (cost > best_cost) {
            best_cost = cost;
            best_v = v;
            best_g = g;
            best_traj = traj;
          }
        }
      }
    }
  }

  // Update state for next iteration
  last_gamma_ = best_g;
  last_v_ = best_v;
  last_direction_ = (best_v >= 0) ? 1 : -1;

  // Visualize best trajectory (cyan)
  if (!best_traj.empty()) {
    visualization_msgs::msg::Marker mb;
    mb.header.frame_id = "map";
    mb.header.stamp = now;
    mb.ns = "best_traj";
    mb.id = id++;
    mb.type = mb.LINE_STRIP;
    mb.action = mb.ADD;
    mb.scale.x = 0.04;
    mb.color.r = 0;
    mb.color.g = 1;
    mb.color.b = 1;
    mb.color.a = 1;
    for (auto &p : best_traj) {
      geometry_msgs::msg::Point P;
      P.x = p.x;
      P.y = p.y;
      P.z = 0;
      mb.points.push_back(P);
    }
    ma.markers.push_back(mb);
  }

  // Visualize local goal (purple)
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = now;
  goal_marker.ns = "local_goal";
  goal_marker.id = id++;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = local_goal.first;
  goal_marker.pose.position.y = local_goal.second;
  goal_marker.pose.position.z = 0;
  goal_marker.scale.x = 0.2;
  goal_marker.scale.y = 0.2;
  goal_marker.scale.z = 0.2;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 1.0;
  goal_marker.color.a = 1.0;
  ma.markers.push_back(goal_marker);

  marker_pub_->publish(ma);

  // Publish commands
  if (!best_traj.empty()) {
    std_msgs::msg::Float64 sp, st;
    sp.data = std::clamp(best_v / V_MAX * MAX_RAW, -MAX_RAW, MAX_RAW);
    
    double steer_angle = (best_v >= 0) ? -best_g : best_g;
    double span = SERVO_MAX - SERVO_C;
    st.data = std::clamp(SERVO_C + (steer_angle / GAMMA_MAX) * span,
                        SERVO_MIN, SERVO_MAX);
    
    speed_pub_->publish(sp);
    steer_pub_->publish(st);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaGlobalFollower>());
  rclcpp::shutdown();
  return 0;
}

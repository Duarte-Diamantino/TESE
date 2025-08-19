#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <functional>
#include <mutex>
#include <memory>
#include <iostream>

struct Pt { double x, y; };

class DwaFollower : public rclcpp::Node {
public:
  DwaFollower();

private:
  // State variables
  double x_, y_, yaw_;
  double goal_x_, goal_y_;
  bool have_odom_;
  bool have_goal_;
  bool joy_override_;
  double last_gamma_;
  double last_v_;
  int last_direction_;
  std::vector<Pt> obstacles_;
  mutable std::mutex obstacles_mutex_;
  double min_safe_distance_;

  // Parameters
  double w_goal_;
  double w_obs_;
  double w_gamma_;
  double horizon_s_;
  int first_level_samples_;
  int second_level_samples_;

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
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  
  // Helper functions
  std::vector<Pt> simulateTrajectory(double v, double g, int steps) const;
  bool isTrajectorySafe(const std::vector<Pt>& traj) const;
  double computeCost(const std::vector<Pt>& traj, double v, double g) const;
  void update();
};

DwaFollower::DwaFollower()
: Node("dwa_follower"),
  x_(0), y_(0), yaw_(0),
  goal_x_(0), goal_y_(0),
  have_odom_(false), have_goal_(false),
  joy_override_(false), last_gamma_(0),
  last_v_(0), last_direction_(1),
  min_safe_distance_(0.5),
  w_goal_(10.0), w_obs_(15.0), w_gamma_(0.5),
  horizon_s_(2.0),
  first_level_samples_(6),
  second_level_samples_(7)
{
  // Declare parameters
  declare_parameter("w_goal", w_goal_);
  declare_parameter("w_obs", w_obs_);
  declare_parameter("w_gamma", w_gamma_);
  declare_parameter("horizon_s", horizon_s_);
  declare_parameter("min_safe_distance", min_safe_distance_);
  declare_parameter("first_level_samples", first_level_samples_);
  declare_parameter("second_level_samples", second_level_samples_);

  // Get parameters
  get_parameter("w_goal", w_goal_);
  get_parameter("w_obs", w_obs_);
  get_parameter("w_gamma", w_gamma_);
  get_parameter("horizon_s", horizon_s_);
  get_parameter("min_safe_distance", min_safe_distance_);
  get_parameter("first_level_samples", first_level_samples_);
  get_parameter("second_level_samples", second_level_samples_);

  // Subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&DwaFollower::odomCb, this, std::placeholders::_1));
  
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&DwaFollower::joyCb, this, std::placeholders::_1));
  
  obstacles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/obstacles_marker", 10,
    std::bind(&DwaFollower::obstaclesCb, this, std::placeholders::_1));

  // Publishers
  speed_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
  steer_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_trajectories", 10);

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(DT * 1000)),
    std::bind(&DwaFollower::update, this));
    
  RCLCPP_INFO(get_logger(), "DWA Follower inicializado!");
}

void DwaFollower::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  auto& q = msg->pose.pose.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw_ = std::atan2(siny_cosp, cosy_cosp);
  have_odom_ = true;
}

void DwaFollower::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy_override_ = std::any_of(
    msg->buttons.begin(), msg->buttons.end(), [](int b){return b>0;});
  if (joy_override_) {
    RCLCPP_WARN(get_logger(), "Joy override ativado!");
  }
}

void DwaFollower::obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
  bool goal_updated = false;
  
  for (const auto& marker : msg->markers) {
    if (marker.ns == "obstacle" || marker.ns == "obstacles") {
      obstacles_.push_back({marker.pose.position.x, marker.pose.position.y});
    }
    else if (marker.ns == "goal") {
      goal_x_ = marker.pose.position.x;
      goal_y_ = marker.pose.position.y;
      goal_updated = true;
    }
  }
  
  if (goal_updated) {
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "Objetivo atualizado: (%.2f, %.2f)", goal_x_, goal_y_);
  }
  
  RCLCPP_INFO(get_logger(), "%zu obstáculos recebidos", obstacles_.size());
}

std::vector<Pt> DwaFollower::simulateTrajectory(double v, double g, int steps) const {
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

bool DwaFollower::isTrajectorySafe(const std::vector<Pt>& traj) const {
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

double DwaFollower::computeCost(const std::vector<Pt>& traj, 
                                     double v, double g) const {
  double cost = 0.0;
  
  // 1. Custo de distância ao objetivo
  double dx_end = traj.back().x - goal_x_;
  double dy_end = traj.back().y - goal_y_;
  double dist_to_goal = std::hypot(dx_end, dy_end);
  cost += -w_goal_ * dist_to_goal;
  
  // 2. Custo de proximidade a obstáculos
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
  
  // Penalidade exponencial para obstáculos próximos
  if (min_obstacle_dist < min_safe_distance_ * 2.0) {
    cost += -w_obs_ / (min_obstacle_dist + 0.01);
  } else {
    cost += w_obs_ * min_obstacle_dist;
  }
  
  // 3. Orientação para o objetivo
  double target_yaw = std::atan2(goal_y_ - y_, goal_x_ - x_);
  double yaw_error = std::abs(yaw_ - target_yaw);
  if (yaw_error > M_PI) {
    yaw_error = 2*M_PI - yaw_error;
  }
  cost += -2.0 * yaw_error;
  
  // 4. Penalidade por mudança de direção
  if ((v >= 0) != (last_v_ >= 0)) {
    cost -= 1000.0;
  }
  
  // 5. Penalidade por mudança brusca de esterço
  cost -= w_gamma_ * std::abs(g - last_gamma_);
  
  // 6. Recompensa por velocidade
  cost += 0.1 * std::abs(v);
  
  return cost;
}

void DwaFollower::update() {
  if(!have_odom_ || !have_goal_ || joy_override_) {
    if (!have_odom_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Esperando por odometria...");
    if (!have_goal_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Esperando por objetivo...");
    if (joy_override_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Joy override ativo");
    return;
  }

  // Verificar se alcançou o objetivo
  double dist_to_goal = std::hypot(x_ - goal_x_, y_ - goal_y_);
  if (dist_to_goal < GOAL_TOL) {
    std_msgs::msg::Float64 stop_speed;
    stop_speed.data = 0.0;
    speed_pub_->publish(stop_speed);
    RCLCPP_INFO(get_logger(), "Objetivo alcançado! Parando.");
    return;
  }

  // Preparar visualização
  visualization_msgs::msg::MarkerArray ma;
  auto now = this->now();
  int id = 0;

  // 1. Marcador do robô (verde)
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

  // 2. Marcador do objetivo (ROSA)
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = now;
  goal_marker.ns = "goal";
  goal_marker.id = id++;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = goal_x_;
  goal_marker.pose.position.y = goal_y_;
  goal_marker.pose.position.z = 0.0;
  goal_marker.scale.x = 0.4;
  goal_marker.scale.y = 0.4;
  goal_marker.scale.z = 0.4;
  goal_marker.color.r = 1.0; // Rosa: R=1, G=0, B=1
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 1.0;
  goal_marker.color.a = 1.0;
  ma.markers.push_back(goal_marker);

  // 3. Marcadores para obstáculos (quadrados BRANCOS)
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      visualization_msgs::msg::Marker obs_marker;
      obs_marker.header.frame_id = "map";
      obs_marker.header.stamp = now;
      obs_marker.ns = "obstacles";
      obs_marker.id = id++;
      obs_marker.type = visualization_msgs::msg::Marker::CUBE;
      obs_marker.action = visualization_msgs::msg::Marker::ADD;
      obs_marker.pose.position.x = obstacles_[i].x;
      obs_marker.pose.position.y = obstacles_[i].y;
      obs_marker.pose.position.z = 0.0;
      obs_marker.scale.x = 0.3;
      obs_marker.scale.y = 0.3;
      obs_marker.scale.z = 0.3;
      obs_marker.color.r = 1.0;
      obs_marker.color.g = 1.0;
      obs_marker.color.b = 1.0;
      obs_marker.color.a = 1.0;
      ma.markers.push_back(obs_marker);
    }
  }

  int steps = static_cast<int>(horizon_s_ / DT);

  // 4. Primeiro nível: direções principais
  std::vector<std::tuple<double, double, double, std::vector<Pt>>> first_level;
  std::vector<std::pair<double, double>> directions = {
    {V_MAX, 0.0}, {V_MAX, GAMMA_MAX/2}, {V_MAX, -GAMMA_MAX/2},
    {V_MAX/2, 0.0}, {V_MAX/2, GAMMA_MAX/2}, {V_MAX/2, -GAMMA_MAX/2}
  };

  int safe_trajectories = 0;
  for(int i = 0; i < first_level_samples_; ++i) {
    double v = directions[i].first;
    double g = directions[i].second;
    auto traj = simulateTrajectory(v, g, steps);
    
    if (!isTrajectorySafe(traj)) {
      // Visualizar trajetórias inseguras em vermelho
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
      m.color.a = 0.3;
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
    
    safe_trajectories++;
    double cost = computeCost(traj, v, g);
    first_level.emplace_back(cost, v, g, traj);
    
    // Visualizar trajetórias seguras do primeiro nível em amarelo
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "first_level";
    m.id = id++;
    m.type = m.LINE_STRIP;
    m.action = m.ADD;
    m.scale.x = 0.03;
    m.color.r = 1.0;
    m.color.g = 1.0;
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
  }

  // Se não houve trajetórias seguras, tentar marcha ré
  if (safe_trajectories == 0) {
    RCLCPP_WARN(get_logger(), "Nenhuma trajetória segura encontrada, tentando marcha ré");
    std::vector<std::pair<double, double>> reverse_directions = {
      {-V_MAX/2, 0.0}, {-V_MAX/2, GAMMA_MAX/2}, {-V_MAX/2, -GAMMA_MAX/2}
    };
    
    for (const auto& dir : reverse_directions) {
      double v = dir.first;
      double g = dir.second;
      auto traj = simulateTrajectory(v, g, steps);
      
      if (!isTrajectorySafe(traj)) continue;
      
      safe_trajectories++;
      double cost = computeCost(traj, v, g);
      first_level.emplace_back(cost, v, g, traj);
    }
  }

  // Se ainda não há trajetórias seguras, parar o robô
  if (safe_trajectories == 0) {
    RCLCPP_ERROR(get_logger(), "NENHUMA trajetória segura encontrada! Parando.");
    std_msgs::msg::Float64 stop_speed;
    stop_speed.data = 0.0;
    speed_pub_->publish(stop_speed);
    marker_pub_->publish(ma);
    return;
  }

  // Ordenar por custo (melhor primeiro)
  std::sort(first_level.begin(), first_level.end(), 
    [](const auto& a, const auto& b){ return std::get<0>(a) > std::get<0>(b); });

  // 5. Segundo nível: refinamento
  double best_cost = -1e9;
  double best_v = 0;
  double best_g = 0;
  std::vector<Pt> best_traj;

  for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
    double base_v = std::get<1>(first_level[i]);
    double base_g = std::get<2>(first_level[i]);
    
    double v_range = (base_v > 0) ? V_MAX/3 : V_MAX/4;
    double g_range = GAMMA_MAX/3;
    
    double dv = 2 * v_range / (second_level_samples_ - 1);
    double dg = 2 * g_range / (second_level_samples_ - 1);
    
    for(int iv = 0; iv < second_level_samples_; ++iv) {
      double v = std::clamp(base_v - v_range + iv * dv, -V_MAX, V_MAX);
      for(int ig = 0; ig < second_level_samples_; ++ig) {
        double g = std::clamp(base_g - g_range + ig * dg, -GAMMA_MAX, GAMMA_MAX);
        
        auto traj = simulateTrajectory(v, g, steps);
        
        if (!isTrajectorySafe(traj)) continue;
        
        double cost = computeCost(traj, v, g);
        
        // Visualizar trajetórias do segundo nível em verde
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

  // Atualizar estado para próxima iteração
  last_gamma_ = best_g;
  last_v_ = best_v;
  last_direction_ = (best_v >= 0) ? 1 : -1;

  // 6. Visualizar melhor trajetória em ciano
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
    
    RCLCPP_INFO(get_logger(), "Melhor comando: v=%.2f m/s, γ=%.2f rad, custo=%.2f", best_v, best_g, best_cost);
  }

  marker_pub_->publish(ma);

  // Publicar comandos
  if (!best_traj.empty()) {
    std_msgs::msg::Float64 sp, st;
    sp.data = std::clamp(best_v / V_MAX * MAX_RAW, -MAX_RAW, MAX_RAW);
    
    double steer_angle = (best_v >= 0) ? -best_g : best_g;
    double span = SERVO_MAX - SERVO_C;
    st.data = std::clamp(SERVO_C + (steer_angle / GAMMA_MAX) * span,
                        SERVO_MIN, SERVO_MAX);
    
    speed_pub_->publish(sp);
    steer_pub_->publish(st);
    
    RCLCPP_DEBUG(get_logger(), "Publicando: speed=%.2f, steer=%.2f", sp.data, st.data);
  } else {
    RCLCPP_WARN(get_logger(), "Nenhuma trajetória válida encontrada!");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaFollower>());
  rclcpp::shutdown();
  return 0;
}
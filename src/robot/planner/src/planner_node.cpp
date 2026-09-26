#include "planner_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PlannerNode::PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger()))
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    is_goal_received_ = true;
    goal_ = *msg;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath(); // can start planning immediately - validity checks in planPath function
    RCLCPP_INFO(this->get_logger(), "goal received, state -> WAITING_FOR_ROBOT_TO_REACH_GOAL");
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  is_odom_received_ = true;
  robot_pose_ = msg->pose.pose;

  //RCLCPP_INFO(this->get_logger(), "planner odom position: %f ",msg->pose.pose.position.x);
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){
    planPath(); //new map comes through and robot in progress
  }
  //RCLCPP_INFO(this->get_logger(), "map returned with %zu cells", msg->data.size()); 
}

void PlannerNode::timerCallback(){
    if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < GOAL_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "goal reached, state -> WAITING_FOR_GOAL");
        state_ = State::WAITING_FOR_GOAL;
    } else {
        RCLCPP_INFO(this->get_logger(), "not yet at goal (dist=%f)", dist);
        planPath(); // replan might be stuck somewhere and no new map coming through
    }
}

void PlannerNode::planPath(){
    if (!is_goal_received_ || !is_odom_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "cannot plan: missing goal, odom, or map data");
        return;
    }
    
    // map metadata
    const auto &info = current_map_.info;

    // convert robot pose and goal point into grid cell indices
    // g: grid
    int start_gx = static_cast<int>((robot_pose_.position.x - info.origin.position.x) / info.resolution);
    int start_gy = static_cast<int>((robot_pose_.position.y - info.origin.position.y) / info.resolution);
    int start_idx = start_gy * info.width + start_gx;

    int goal_gx = static_cast<int>((goal_.point.x - info.origin.position.x) / info.resolution);
    int goal_gy = static_cast<int>((goal_.point.y - info.origin.position.y) / info.resolution);
    int goal_idx = goal_gy * info.width + goal_gx;

    // call A* search and return list of cells that form path
    std::vector<int> path_cells = planner_.searchAStar(current_map_.data, start_idx, goal_idx);

    if (path_cells.empty()) {
        RCLCPP_WARN(this->get_logger(), "no path found from start to goal");
        return;
    }

    // initialise array of pairs to store world coordinates 
    std::vector<std::pair<double, double>> world_points;
    world_points.reserve(path_cells.size());

    // convert each grid cell to world cooridinate
    // g: grid cell, w: world
    for (int idx : path_cells) {
        int gx = idx % info.width;
        int gy = idx / info.width;
        double wx = info.origin.position.x + (gx + 0.5) * info.resolution;
        double wy = info.origin.position.y + (gy + 0.5) * info.resolution;
        world_points.push_back({wx, wy});
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "sim_world";

    // calculate heading to next point

    double prev_yaw = 0.0; // if the goal point is at the robot position

    for (size_t i = 0; i < world_points.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = world_points[i].first;
        pose_stamped.pose.position.y = world_points[i].second;
        pose_stamped.pose.position.z = 0.0;

        double yaw;
        if (i + 1 < world_points.size()) {
            double dx = world_points[i + 1].first - world_points[i].first;
            double dy = world_points[i + 1].second - world_points[i].second;
            yaw = std::atan2(dy, dx); // accounts for different quadrants
            prev_yaw = yaw;
        } else {
            yaw = prev_yaw; // last point same heading as penultimate
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // construct quarternion - only yaw
        pose_stamped.pose.orientation = tf2::toMsg(q); // convert to pose msg format

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

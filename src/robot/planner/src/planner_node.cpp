#include "planner_node.hpp"

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
    
    RCLCPP_INFO(this->get_logger(), "goal received, state -> WAITING_FOR_ROBOT_TO_REACH_GOAL");
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  is_odom_received_ = true;
  robot_pose_ = msg->pose.pose;

  RCLCPP_INFO(this->get_logger(), "planner odom position: %f ",msg->pose.pose.position.x);
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  RCLCPP_INFO(this->get_logger(), "map returned with %zu cells", msg->data.size()); 
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
        // planPath() 
    }
}

void PlannerNode::planPath(){
    if (!is_goal_received_ || !is_odom_received_ || current_map.data.empty()) {
        //warning message
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

    //convert path cells to world coordinates and calculate heading

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";
    path_msg.poses = //list of poses with orientation

    path_pub_->publish(path_msg)

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

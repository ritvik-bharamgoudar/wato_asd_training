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
    goal_ = *msg;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    
    RCLCPP_INFO(this->get_logger(), "goal received, state -> WAITING_FOR_ROBOT_TO_REACH_GOAL");
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
  RCLCPP_INFO(this->get_logger(), "planner odom position: %f ",msg->pose.pose.position.x);
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "map returned with %zu cells", msg->data.size()); 
}

void PlannerNode::timerCallback(){
    if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 0.5) {
        RCLCPP_INFO(this->get_logger(), "goal reached, state -> WAITING_FOR_GOAL");
        state_ = State::WAITING_FOR_GOAL;
    } else {
        RCLCPP_INFO(this->get_logger(), "not yet at goal (dist=%f)", dist);
        // planPath() 
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

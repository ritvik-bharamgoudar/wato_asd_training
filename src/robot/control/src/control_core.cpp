#include "control_core.hpp"
#include "control_core.hpp"

#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1,2) + std::pow(y2 - y1,2));
}

double ControlCore::computeHeadingError(double lx, double ly) 
{
    return std::atan2(ly, lx); // heading to lookahead point
}

// finds first path node above dist threshold and converts to robot frame
std::tuple<double, double, bool, int> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr path,
    double robot_x, double robot_y, double robot_theta,
    double lookahead_distance, int start_index)
{

    double cos_t = std::cos(-robot_theta);
    double sin_t = std::sin(-robot_theta);

    for (int i = start_index; i < static_cast<int>(path->poses.size()); ++i) {
        double px = path->poses[i].pose.position.x;
        double py = path->poses[i].pose.position.y;
        double dist = computeDistance(robot_x, robot_y, px, py);

        if (dist >= lookahead_distance) {
            // vector path robot to path node
            double dx = px - robot_x;
            double dy = py - robot_y;
            // where is path node relative to robot (inverse transform)
            double lx = dx * cos_t - dy * sin_t;
            double ly = dx * sin_t + dy * cos_t;
            return std::make_tuple(lx, ly, true, i);
        }
    }
    // if nothing found then must be too close to goal
    // set lookahead to goal point
    //f:final node, d: delta, l: lookahead
    double fx = path->poses.back().pose.position.x;
    double fy = path->poses.back().pose.position.y;
    double dx = fx - robot_x;
    double dy = fy - robot_y;
    double lx = dx * cos_t - dy * sin_t;
    double ly = dx * sin_t + dy * cos_t;
    int final_index = static_cast<int>(path->poses.size()) - 1;
    return std::make_tuple(lx, ly, true, final_index);
}


std::pair<double, double> ControlCore::purePursuit(
    double lx, double ly, double linear_speed, double max_angular_z)
{

    // in case lookahead point too close to robot
    if ((std::pow(lx,2) + (std::pow(ly,2))) < 1e-6) {
        return std::make_pair(0.0, 0.0);
    }

    // x0,y0 = 0,0 therefore, (lx^2) + (ly^2 - r^2) = r^2
    double curvature = 2.0 * ly / ((std::pow(lx,2)) + (std::pow(ly,2)));
    double linear_x = linear_speed;
    double angular_z = curvature * linear_x; // curvature = 1/r

    if (std::abs(angular_z) > max_angular_z) {
        linear_x = max_angular_z / std::abs(curvature);
        angular_z = curvature * linear_x;
    }

    return std::make_pair(linear_x, angular_z);
}

// called in node if acute heading to next point
std::pair<double, double> ControlCore::turnInPlace(double heading_error, double rotate_angular_speed)
{   
    //which way to turn
    double angular_z = (heading_error > 0.0) ? rotate_angular_speed : -rotate_angular_speed;
    return std::make_pair(0.0, angular_z);
}

double ControlCore::rateLimit(double desired_vel, double last_vel, double max_delta_vel) 
{
    double delta = desired_vel - last_vel;
    if (delta > max_delta_vel) {
        delta = max_delta_vel;
    } else if (delta < -max_delta_vel) {
        delta = -max_delta_vel;
    }
    return last_vel + delta;
}

bool ControlCore::checkGoalReached(const nav_msgs::msg::Path::SharedPtr path, double robot_x, double robot_y, double goal_tolerance)
{
    const auto& goal_pose = path->poses.back(); //last path node
    double dist = computeDistance(robot_x, robot_y, goal_pose.pose.position.x, goal_pose.pose.position.y);
    return dist <= goal_tolerance;
}

}




#include "opera_nav2_route_plugins/route_operations/opera_route_visualizer.hpp"

#include <string>

namespace opera_nav2_route_plugins
{

void OperaRouteVisualizer::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> /* costmap_subscriber */,
    const std::string & name)
{
    name_ = name;
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(
        node, getName() + ".width_tag", rclcpp::ParameterValue("width"));
    width_tag_ = node->get_parameter(getName() + ".width_tag").as_string();

    nav2_util::declare_parameter_if_not_declared(
        node, getName() + ".grade_tag", rclcpp::ParameterValue("grade"));
    grade_tag_ = node->get_parameter(getName() + ".grade_tag").as_string();

    nav2_util::declare_parameter_if_not_declared(
        node, getName() + ".tags_tag", rclcpp::ParameterValue("tags"));
    tags_tag_ = node->get_parameter(getName() + ".tags_tag").as_string();

    width_pub_ = node->create_publisher<std_msgs::msg::Float64>(width_tag_, 1); 
    grade_pub_ = node->create_publisher<std_msgs::msg::Float64>(grade_tag_, 1);
    width_pub_->on_activate();
    grade_pub_->on_activate();
}


nav2_route::OperationResult OperaRouteVisualizer::perform(
    nav2_route::NodePtr /*node*/,
    nav2_route::EdgePtr edge_entered,
    nav2_route::EdgePtr /*edge_exited*/,
    const nav2_route::Route & /*route*/,
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const nav2_route::Metadata * /*mdata*/) 
{
    nav2_route::OperationResult result;
    if (!edge_entered) {
        return result;
    }

    float width = 3.0;
    width = edge_entered->metadata.getValue<float>(width_tag_, width);
    float grade = 0.0;
    grade = edge_entered->metadata.getValue<float>(grade_tag_, grade);

    auto msg_width = std::make_unique<std_msgs::msg::Float64>();
    msg_width->data = width;
    width_pub_->publish(std::move(msg_width));

    auto msg_grade = std::make_unique<std_msgs::msg::Float64>();
    msg_grade->data = grade;
    grade_pub_->publish(std::move(msg_grade));
    
    return result;
}


}  // namespace opera_nav2_route_plugins


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opera_nav2_route_plugins::OperaRouteVisualizer, nav2_route::RouteOperation)
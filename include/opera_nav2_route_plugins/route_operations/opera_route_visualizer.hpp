

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "std_msgs/msg/float64.hpp"

#include <string>


namespace opera_nav2_route_plugins
{

class OperaRouteVisualizer : public nav2_route::RouteOperation
{

public:
    OperaRouteVisualizer() = default;

    virtual ~OperaRouteVisualizer() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
        const std::string & name) override;
    
    std::string getName() override {return name_;}

    nav2_route::RouteOperationType processType() override {return nav2_route::RouteOperationType::ON_STATUS_CHANGE;}

    nav2_route::OperationResult perform(
        nav2_route::NodePtr /*node*/,
        nav2_route::EdgePtr curr_edge,
        nav2_route::EdgePtr /*edge_exited*/,
        const nav2_route::Route & route,
        const geometry_msgs::msg::PoseStamped & curr_pose,
        const nav2_route::Metadata * /*mdata*/) override;

protected:
    std::string name_;
    std::string width_tag_;
    std::string grade_tag_;
    std::string tags_tag_;

    rclcpp::Logger logger_{rclcpp::get_logger("OperaRouteVisualizer")};
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr width_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr grade_pub_;
};

}  // namespace opera_nav2_route_plugins

// Copyright (c) 2025 Joshua Wallace
// 2026 itsuka-to
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "opera_nav2_route_plugins/graph_file_loaders/opera_geojson_graph_file_loader.hpp"

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

namespace nav2_route
{

void OperaGeoJsonGraphFileLoader::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring OPERA geojson graph file loader");
  logger_ = node->get_logger();
  node_ = node;

  route_frame_ = node_->get_parameter("route_frame").as_string();

  markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("area", 1);
  markers_pub_->on_activate();

  get_area_service_ = node->create_service<opera_bt_msgs::srv::GetArea>("get_area", 
    std::bind(
      &OperaGeoJsonGraphFileLoader::getAreaCallback, this, std::placeholders::_1, std::placeholders::_2
    ));
}

bool OperaGeoJsonGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{ 
  if (!doesFileExist(filepath)) {
    RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
    return false;
  }

  std::ifstream graph_file(filepath);
  Json json_graph;

  try {
    json_graph = Json::parse(graph_file);
  } catch (Json::parse_error & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse %s: %s", filepath.c_str(), ex.what());
    return false;
  }

  auto features = json_graph["features"];
  std::vector<Json> nodes, edges;
  getGraphElements(features, nodes, edges, areas_);

  if (nodes.empty() || edges.empty()) {
    RCLCPP_ERROR(
      logger_, "The graph is malformed. Is does not contain nodes or edges. Please check %s",
      filepath.c_str());
    return false;
  }

  graph.resize(nodes.size());
  addNodesToGraph(graph, graph_to_id_map, nodes);
  addEdgesToGraph(graph, graph_to_id_map, edges);

  // polygon
  visualization_msgs::msg::MarkerArray markers_msg;
  for (const auto & polygon : areas_){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = route_frame_;
    marker.header.stamp = node_->now();

    std::string name = polygon["properties"].value("name", "polygon");
    marker.ns = name;
    int id = polygon["properties"].value("id", 0);
    marker.id = id;

    const auto & ring = polygon["geometry"].at("coordinates").at(0);
    for (const auto & coord : ring) {
      geometry_msgs::msg::Point p;
      p.x = coord.at(0).get<double>();
      p.y = coord.at(1).get<double>();
      p.z = 0.0;

      marker.points.push_back(p);
    }

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    markers_msg.markers.push_back(marker);
  }

  markers_pub_->publish(markers_msg);

  return true;
}


void OperaGeoJsonGraphFileLoader::getAreaCallback(
  const std::shared_ptr<opera_bt_msgs::srv::GetArea::Request> request,
  std::shared_ptr<opera_bt_msgs::srv::GetArea::Response> response)
{
  response->area_id = 0;

  if (areas_.size() == 0){
    RCLCPP_ERROR(logger_, "[getAreaCallback] areas_.len() == 0.");
    return;
  }

  if (request->pose.header.frame_id != std::string("map")){
    RCLCPP_ERROR(logger_, "[getAreaCallback] request->pose.header.frame_id != 'map'");
    return;
  }

  for (const auto & area : areas_){
    Polygon polygon;

    for (const auto & point : area["geometry"]["coordinates"][0]){
      polygon.outer().push_back(Point(point[0], point[1]));
    }
    bg::correct(polygon);

    Point p(request->pose.pose.position.x, request->pose.pose.position.y);

    if (bg::within(p, polygon)){

      unsigned int area_id = area["properties"]["id"];
      RCLCPP_INFO(logger_, "[getAreaCallback] your area is %d", area_id);
      response->area_id = area_id;
      return;
    }
  }
}


bool OperaGeoJsonGraphFileLoader::doesFileExist(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}

void OperaGeoJsonGraphFileLoader::getGraphElements(
  const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges, std::vector<Json> & polygons)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    } else if (  // NOLINT
      feature["geometry"]["type"] == "MultiLineString" ||
      feature["geometry"]["type"] == "LineString")
    {
      edges.emplace_back(feature);
    } else if (
      feature["geometry"]["type"] == "Polygon"
    )
    {
      polygons.emplace_back(feature);
    }
  } 
}

void OperaGeoJsonGraphFileLoader::addNodesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes)
{
  int idx = 0;
  for (const auto & node : nodes) {
    const auto properties = node["properties"];
    graph[idx].nodeid = properties["id"];
    graph_to_id_map[graph[idx].nodeid] = idx;
    graph[idx].coords = convertCoordinatesFromJson(node);
    graph[idx].operations = convertOperationsFromJson(properties);
    graph[idx].metadata = convertMetaDataFromJson(properties);
    idx++;
  }
}

void OperaGeoJsonGraphFileLoader::addEdgesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges)
{
  for (const auto & edge : edges) {
    // Required data
    const auto properties = edge["properties"];
    unsigned int id = properties["id"];
    unsigned int start_id = properties["startid"];
    unsigned int end_id = properties["endid"];

    if (graph_to_id_map.find(start_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "Start id %u does not exist for edge id %u", start_id, id);
      throw nav2_core::NoValidGraph("Start id does not exist");
    }

    if (graph_to_id_map.find(end_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "End id of %u does not exist for edge id %u", end_id, id);
      throw nav2_core::NoValidGraph("End id does not exist");
    }

    EdgeCost edge_cost = convertEdgeCostFromJson(properties);
    Operations operations = convertOperationsFromJson(properties);
    Metadata metadata = convertMetaDataFromJson(properties);

    graph[graph_to_id_map[start_id]].addEdge(
      edge_cost, &graph[graph_to_id_map[end_id]], id,
      metadata, operations);
  }
}

Coordinates OperaGeoJsonGraphFileLoader::convertCoordinatesFromJson(const Json & node)
{
  Coordinates coords;
  const auto & properties = node["properties"];
  if (properties.contains("frame")) {
    coords.frame_id = properties["frame"];
  }

  const auto & coordinates = node["geometry"]["coordinates"];
  coords.x = coordinates[0];
  coords.y = coordinates[1];

  return coords;
}

Metadata OperaGeoJsonGraphFileLoader::convertMetaDataFromJson(
  const Json & properties,
  const std::string & key)
{
  Metadata metadata;
  if (!properties.contains(key)) {return metadata;}

  for (const auto & data : properties[key].items()) {
    if (data.value().is_object() ) {
      Metadata new_metadata = convertMetaDataFromJson(properties[key], data.key());
      metadata.setValue(data.key(), new_metadata);
      continue;
    }

    const auto setPrimitiveType = [&](const auto & value) -> std::any
      {
        if (value.is_number()) {
          if (value.is_number_unsigned()) {
            return static_cast<unsigned int>(value);
          } else if (value.is_number_integer()) {
            return static_cast<int>(value);
          } else {
            return static_cast<float>(value);
          }
        }

        if (value.is_boolean()) {
          return static_cast<bool>(value);
        }

        if (value.is_string()) {
          return static_cast<std::string>(value);
        }
        RCLCPP_ERROR(
          logger_, "Failed to convert the key: %s to a value", data.key().c_str());
        throw std::runtime_error("Failed to convert");
      };

    if (data.value().is_array()) {
      std::vector<std::any> array;
      for (const auto & el : data.value()) {
        auto value = setPrimitiveType(el);
        array.push_back(value);
      }
      metadata.setValue(data.key(), array);
      continue;
    }

    auto value = setPrimitiveType(data.value());
    metadata.setValue(data.key(), value);
  }

  return metadata;
}

Operation OperaGeoJsonGraphFileLoader::convertOperationFromJson(const Json & json_operation)
{
  Operation operation;
  json_operation.at("type").get_to(operation.type);
  Json trigger = json_operation.at("trigger");
  operation.trigger = trigger.get<OperationTrigger>();
  Metadata metadata = convertMetaDataFromJson(json_operation);
  operation.metadata = metadata;

  return operation;
}

Operations OperaGeoJsonGraphFileLoader::convertOperationsFromJson(const Json & properties)
{
  Operations operations;
  if (properties.contains("operations")) {
    for (const auto & json_operation : properties["operations"]) {
      operations.push_back(convertOperationFromJson(json_operation));
    }
  }
  return operations;
}

EdgeCost OperaGeoJsonGraphFileLoader::convertEdgeCostFromJson(const Json & properties)
{
  EdgeCost edge_cost;
  if (properties.contains("cost")) {
    edge_cost.cost = properties["cost"];
  }

  if (properties.contains("overridable")) {
    edge_cost.overridable = properties["overridable"];
  }
  return edge_cost;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::OperaGeoJsonGraphFileLoader, nav2_route::GraphFileLoader)

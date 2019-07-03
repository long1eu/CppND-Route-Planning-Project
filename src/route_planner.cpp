#include "route_planner.h"
#include <algorithm>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model), m_startNode{nullptr}, m_endNode{nullptr}, m_distance{std::numeric_limits<float>::max()} {
  const float SCALE_FACTOR = 0.01;
  start_x *= SCALE_FACTOR;
  start_y *= SCALE_FACTOR;
  end_x *= SCALE_FACTOR;
  end_y *= SCALE_FACTOR;

  m_startNode = &m_Model.FindClosestNode(start_x, start_y);
  m_endNode = &m_Model.FindClosestNode(end_x, end_y);
}

vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  RouteModel::Node *parent = nullptr;
  m_distance = 0.0f;
  vector<RouteModel::Node> found_path{};
  if (current_node == nullptr) {
    std::cout << "ERROR: Nullpr passed, exiting" << std::endl;
    return found_path;
  }

  found_path.emplace_back(*current_node);
  while ((parent = current_node->parent) != nullptr) {
    found_path.emplace_back(*parent);
    m_distance += current_node->Distance(*parent);
    current_node = parent;
  }
  m_distance *= m_Model.MetricScale();
  return found_path;
}

void RoutePlanner::AStarSearch() {
  m_startNode->visited = true;
  m_openList.push_back(m_startNode);
  RouteModel::Node *current_node = nullptr;
  while (!m_openList.empty()) {
    current_node = NextNode();
    if (current_node->Distance(*m_endNode) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    } else {
      AddNeighbors(current_node);
    }
  }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
  return node->Distance(*m_endNode);
}

RouteModel::Node *RoutePlanner::NextNode() {
  RouteModel::Node *next_node = nullptr;
  std::sort(m_openList.begin(), m_openList.end(), [](RouteModel::Node *n1, RouteModel::Node *n2) -> bool {
              return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
            }
  );
  if (!m_openList.empty()) {
    next_node = m_openList.back();
    m_openList.pop_back();
  }
  return next_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  if (current_node == nullptr) {
    std::cout << "RoutePlanner::AddNeighbors, received nullptr, exiting " << std::endl;
  }
  current_node->FindNeighbors();
  for (auto &neighbor_node:current_node->neighbors) {
    neighbor_node->parent = current_node;
    neighbor_node->g_value = current_node->g_value + current_node->Distance(*neighbor_node);
    neighbor_node->h_value = CalculateHValue(neighbor_node);
    m_openList.push_back(neighbor_node);
    neighbor_node->visited = true;
  }
}
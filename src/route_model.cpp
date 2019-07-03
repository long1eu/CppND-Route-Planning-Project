#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int i = 0;
  for (const Model::Node &node:this->Nodes()) {
    m_Nodes.emplace_back(i, this, node);
    i++;
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (const Model::Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx : Ways()[road.way].nodes) {
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = vector<const Model::Road *>();
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  RouteModel::Node node{x, y};
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;
  for (const auto &road:Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int idx:Ways()[road.way].nodes) {
        float dist = node.Distance(SNodes()[idx]);
        if (dist < min_dist) {
          min_dist = dist;
          closest_idx = idx;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}

RouteModel::Node::Node(float x, float y) : RouteModel::Node::Node(0, nullptr, Model::Node{x, y}) {
}

RouteModel::Node::Node(int idx, RouteModel *search_model, Model::Node node) :
    Model::Node(node),
    parent_model{search_model},
    parent{nullptr},
    h_value{std::numeric_limits<float>::max()},
    g_value{0.0},
    visited{false},
    neighbors{},
    index(idx) {
}

RouteModel::Node *RouteModel::Node::FindNeighbor(vector<int> node_indices) const {
  Node *closest_neighbor = nullptr;
  double min_distance = std::numeric_limits<double>::max();
  for (int node_index:node_indices) {
    Node &node = parent_model->m_Nodes[node_index];
    if (!node.visited && Distance(node) != 0) {
      double d = Distance(node);
      if (d < min_distance || closest_neighbor == nullptr) {
        min_distance = d;
        closest_neighbor = &node;
      }
    }
  }
  return closest_neighbor;
}

void RouteModel::Node::FindNeighbors() {
  for (const Road *road:parent_model->node_to_road[this->index]) {
    RouteModel::Node *neighboring_node = nullptr;
    if (parent_model) {
      neighboring_node = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    }
    if (neighboring_node) {
      this->neighbors.push_back(neighboring_node);
    }
  }
}

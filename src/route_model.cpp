#include <iostream>
#include <cmath>
#include "route_model.h"
#include "model.h"

using std::sqrt;
using std::pow;

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {

    int size = Nodes().size();
    for (int i = 0; i < size; i++) {
        Model::Node node = Nodes()[i];

        m_Nodes.emplace_back(i, this, node);
    }

}

float RouteModel::Node::Distance(const RouteModel::Node &node) const {
    return sqrt(pow(x - node.x, 2) + pow(y - node.y, 2));
}

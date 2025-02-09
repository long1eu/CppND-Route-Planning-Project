#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include "../src/route_model.h"
#include "../src/route_planner.h"


static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

std::vector<std::byte> ReadOSMData(const std::string &path) {
    std::vector<std::byte> osm_data;
    auto data = ReadFile(path);
    if( !data ) {
        std::cout << "Failed to read OSM data." << std::endl;
    } else {
        osm_data = std::move(*data);
    }
    return osm_data;
}


class RouteModelTest : public ::testing::Test {
  protected:
    std::string osm_data_file = "../map.osm";
    std::vector<std::byte> osm_data = ReadOSMData(osm_data_file);
    RouteModel model{osm_data};
};


// Test that the path size is zero initially.
TEST_F(RouteModelTest, RouteModelData) {
    EXPECT_EQ(model.path.size(), 0);
    EXPECT_EQ(model.Nodes().size(), model.SNodes().size());
}


// Test that the RouteModel::Node class is defined correctly.
TEST_F(RouteModelTest, RouteModelNode) {
    RouteModel::Node test_node = model.SNodes()[1];
    EXPECT_FLOAT_EQ(test_node.x, 1.2646476);
    EXPECT_FLOAT_EQ(test_node.y, 0.23506972);
    EXPECT_EQ(test_node.parent, nullptr);
    EXPECT_FLOAT_EQ(test_node.h_value, std::numeric_limits<float>::max());
    EXPECT_FLOAT_EQ(test_node.g_value, 0.0);
    EXPECT_FLOAT_EQ(test_node.visited, false);
    EXPECT_EQ(test_node.neighbors.size(), 0);
    RouteModel::Node test_node_2 = RouteModel::Node();
}

// Test the distance function between nodes.
TEST_F(RouteModelTest, NodeDistance) {
    RouteModel::Node test_node = model.SNodes()[1];
    RouteModel::Node test_node_2 = model.SNodes()[4];
EXPECT_FLOAT_EQ(test_node
.
Distance(test_node_2),
0.10309877);
}

// Test the data created by CreateNodeToRoadHashmap.
TEST_F(RouteModelTest, NodeToRoad) {
    auto node_to_road = model.GetNodeToRoadMap();
    EXPECT_EQ(node_to_road[0].size(), 2);
    EXPECT_EQ(node_to_road[30].size(), 2);
    EXPECT_EQ(node_to_road[90].size(), 2);
    EXPECT_EQ(node_to_road[0][0]->way, 500);
    EXPECT_EQ(node_to_road[30][1]->way, 613);
    EXPECT_EQ(node_to_road[90][1]->way, 475);
}

// Test the FindNeighbors method.
TEST_F(RouteModelTest, FindNeighbors) {
    auto test_node = model.SNodes()[0];
    test_node.FindNeighbors();
    EXPECT_EQ(test_node.neighbors.size(), 2);
    EXPECT_FLOAT_EQ(test_node.neighbors[1]->x, 1.3250526);
    EXPECT_FLOAT_EQ(test_node.neighbors[1]->y, 0.41849667);
    test_node.neighbors.clear(); // Clear out neighbors just added.
    test_node = model.SNodes()[100];
    test_node.FindNeighbors();
    EXPECT_EQ(test_node.neighbors.size(), 2);
    EXPECT_FLOAT_EQ(test_node.neighbors[0]->x, 0.77367586);
    EXPECT_FLOAT_EQ(test_node.neighbors[0]->y, 0.52004427);
    test_node.neighbors.clear();
}

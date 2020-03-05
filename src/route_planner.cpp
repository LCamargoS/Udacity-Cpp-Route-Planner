#include "route_planner.h"
#include <algorithm>
#include <iostream>

using std::reverse;
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        if (neighbor->visited == false) {
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->parent = current_node;
            neighbor->visited = true;

            open_list.push_back(neighbor);
        }
    }
}

bool sortOpenNodes(RouteModel::Node* na, RouteModel::Node* nb) 
{
    return (na->g_value + na->h_value) < (nb->g_value + nb->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), sortOpenNodes);
    RouteModel::Node *closest_node = open_list.front();
    open_list.erase(open_list.begin());

    return closest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent) {
        path_found.push_back(*current_node);
        RouteModel::Node *parent = current_node->parent;
        distance = distance += current_node->distance(*parent);
        current_node = parent;
    }

    path_found.push_back(*start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    reverse(path_found.begin(),path_found.end());

    return path_found;
}

void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while(!open_list.empty()) {
        current_node = NextNode();
        float distance = current_node->distance(*end_node);

        if (distance == 0) {
            m_Model.path = ConstructFinalPath(current_node);

            return;
        }

        AddNeighbors(current_node);
    }
}
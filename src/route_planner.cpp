#include "route_planner.h"
#include <algorithm>
#include <cassert>
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    start_node -> visited = true;
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}



// Calculates the h_value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto neighbor : current_node->neighbors){
        if(neighbor->visited == false){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;}
    }

}



RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),[](auto a, auto b){
        return (a->g_value + a->h_value < b->g_value + b->h_value);
    });
    RouteModel::Node* next_node = open_list.front();
    open_list.erase(open_list.begin());
    return next_node;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr){
        auto parent = current_node->parent;
        distance += current_node->distance(*parent);
        path_found.insert(path_found.begin(),(*current_node));
        current_node = current_node->parent;
    }

    path_found.insert(path_found.begin(),(*current_node));
        

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    open_list.push_back(current_node);
    while(open_list.size() > 0){
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0){
            break;
        }
        AddNeighbors(current_node);
        
    }
    m_Model.path = ConstructFinalPath(current_node);
    // TODO: Implement your solution here.

}
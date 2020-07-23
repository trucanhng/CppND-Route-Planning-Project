#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest node to the starting coordinates
    // and store it in the start_node
    start_node = &m_Model.FindClosestNode (start_x, start_y);

    // Find the closest node to the ending coordinates
    // and store it in the end_node
    end_node = &m_Model.FindClosestNode (end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance (*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Find all neighbors of the current node
  current_node->FindNeighbors ();

  for (auto neighbor : current_node->neighbors)
  {// For each neighbor in the current node's neighbor list
    // Set the neighbor's parent to the current node
    neighbor->parent = current_node;
    // Calculate the neighbor's h_value
    neighbor->h_value = CalculateHValue (neighbor);
    // Calculate the neighbor's g_value
    neighbor->g_value = current_node->g_value + neighbor->distance (*current_node);
    // Add neighbor to the open_list
    open_list.push_back (neighbor);
    // Mark neighbor as visited
    neighbor->visited = true;
  }

}

// Compare the sum of the h and g value of 2 nodes
// Return true if node1's sum > node2's sum
bool RoutePlanner::Compare (RouteModel::Node *node1, RouteModel::Node *node2)
{
  float f1 = node1->g_value + node1->h_value;
  float f2 = node2->g_value + node2->h_value;
  return (f1 > f2);
}

// Sort the open_list in descending order
void RoutePlanner::Sort (std::vector<RouteModel::Node *> *list)
{
  std::sort (list->begin(), list->end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list in descending order
  // based on the sum of the h_value and g_value
  Sort (&open_list);

  // Get the next_node, which is the node with the lowest (h + g)
  RouteModel::Node *next_node = open_list.back();

  // Remove the node from the list
  open_list.pop_back();
  return next_node;
}

// - This method takes the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != start_node)
    {
      // If the current_node is not the start_node
      // Add the current_node to the final path
      path_found.push_back(*current_node);
      // Update the total distance
      distance += current_node->distance (*current_node->parent);
      // Move up one node to continue with the path construction
      current_node = current_node->parent;
    }

    // Once the start_node is found, we need to add it to path as well
    path_found.push_back (*start_node);

    // Reverse the whole path order so that
    // the start node is the first element of the vector
    std::reverse (path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters
    distance *= m_Model.MetricScale();

    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Add start node to open_list, and mark it as visited
    open_list.push_back (start_node);
    start_node->visited = true;

    while (!open_list.empty ())
    {
      // Find the next node
      current_node = NextNode ();

      if (current_node == end_node)
      {
        // Construct the final path if we have reached the end node
        m_Model.path = ConstructFinalPath (current_node);
        return;
      }
      else
      {
        // If we have not reached the end node, continue the search
        // with the current node's neighbors
        AddNeighbors (current_node);
      }
    }
}

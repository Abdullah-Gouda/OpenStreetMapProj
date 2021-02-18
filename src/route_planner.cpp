#include "route_planner.h"
#include <algorithm>

static bool Compare(const RouteModel::Node *node_1, const RouteModel::Node *node_2) {
         float f1 = node_1->g_value+node_1->h_value;
        float f2 = node_2->g_value+node_2->h_value;
        return f1 > f2; 
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  // RouteModel::Node &RouteModel::FindClosestNode(float x, float y);
	start_node =  &(m_Model.FindClosestNode(start_x, start_y));
   //std::cout<<"closest node to start_node:  ";
   //std::cout<< start_node->x << "   , " << start_node->y <<std::endl;
  end_node =  &(m_Model.FindClosestNode(end_x, end_y));
  // std::cout<<"closest node to end_node:  ";
  // std::cout<< end_node->x << "   , " << end_node->y <<std::endl;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  // float distance(Node other) const
	float DistanceTOEndNode = node->distance(*end_node);
 // std::cout<<"DistanceTOEndNode = " << DistanceTOEndNode<<std::endl;
  return DistanceTOEndNode;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// 1- Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// 2- For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// 3- Use CalculateHValue below to implement the h-Value calculation.
// 4- For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
// std::vector<Node *> neighbors;
  current_node->FindNeighbors();
  for(int i=0; i<current_node->neighbors.size();i++)
  {
   //std::cout<< current_node->neighbors[i]<<"\n";
    current_node->neighbors[i]->parent = current_node;
    current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);
    // g-value is the distance from start node till thecureent node
    current_node->neighbors[i]->g_value = current_node->neighbors[i]->distance(*start_node);
    this->open_list.push_back(current_node->neighbors[i]);
    current_node->neighbors[i]->visited = true;
  }
  //2- 
  
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_listaccording to the sum of the h value and g value.
// - Crea te a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
     std::sort(open_list.begin(), open_list.end(),Compare);
    RouteModel::Node* lowest_sum;
    lowest_sum = open_list.back();
    this->open_list.pop_back();
    return lowest_sum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
//if the passed node has a parent. If there is a parent, we should push the current_node to the path_found vector and increase the distance value by the distance between it and its parent.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  while(current_node->parent != nullptr)
  	{
    	path_found.push_back(*current_node);
    	distance+= current_node->distance(*(current_node->parent));
    	current_node = current_node->parent;
 	 }
	path_found.push_back(*current_node); // push start node the path, as it has no parent
    // TODO: Implement your solution here.
	 std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	current_node = start_node;
  	current_node->visited = true;
	RoutePlanner::AddNeighbors(current_node);
  while(open_list.size() > 0)
  {
    current_node = NextNode();
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(end_node);
      break;
    }
    else {
         AddNeighbors(current_node);
    }
  }
    // TODO: Implement your solution here.

}
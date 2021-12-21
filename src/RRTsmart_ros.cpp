/*
  RRTsmart_ros.cpp
*/

#include <rrt_smart_global_planner/RRTsmart_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <random>
#include <chrono>
#include <bits/stdc++.h>

#define PI 3.141592653589


using namespace std;

std::random_device rd;
static std::default_random_engine generator(rd());

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRTsmart_planner::RRTsmartPlannerROS, nav_core::BaseGlobalPlanner)

namespace RRTsmart_planner
{

  RRTsmartPlannerROS::RRTsmartPlannerROS()
      : costmap_(nullptr), initialized_(false) {}

  RRTsmartPlannerROS::RRTsmartPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : costmap_ros_(costmap_ros)
  {
    //initialize the planner
    initialize(name, costmap_ros);
  }

  void RRTsmartPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {

    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
      width = costmap_->getSizeInCellsX();
      height = costmap_->getSizeInCellsY();
      resolution = costmap_->getResolution();

      RADIUS = 0.3;       //default 1.0  ha d'anar lligat (més petit) a inflation_radius de commn_costmap
      GOAL_RADIUS = 0.5;  //default 0.5
      epsilon_min = 0.35; //default 0.05
      epsilon_max = 0.35; //default 0.1

      ROS_INFO("RRT-Smart planner initialized successfully");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTsmartPlannerROS::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal,
                                    std::vector<geometry_msgs::PoseStamped> &plan)
  {
    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, std::pair<float, float>>> rrt_smart_path;  // Declaration of variable rrt_smart_path (array)
    std::vector<std::pair<float, std::pair<float, float>>> rrt_star_path;   // Declaration of variable rrt_star_path<cost, <x, y>>  (array)
    std::vector<std::pair<float, std::pair<float, float>>> aux_path;
    std::vector<std::pair<float, float>> node_beacons;

    rrt_smart_path.clear();
    rrt_star_path.clear();
    node_beacons.clear();

    float new_path_cost = 0.0;
    float old_path_cost = 20000.0; // first old path have to be bigger than any new cost could found (condition below)
    old_cost = 20000.0;

    float biasing_radius = 0.25; // radius of biasing circle
    int biasing_ratio = 10;     // each how many iterations sample random point near beacon_node
    int idx_beacon = 0;

	bool path_found = false;

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    std::vector<Node> nodes;
    nodes.clear();

    MAX_NUM_NODES = 5000;  // Number of nodes until the path is given (a més gran més òptim)
    iterations = 0;

    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;

    nodes.push_back(start_node);

    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;

    ROS_INFO("Start: X -> %fm, Y -> %fm", start.pose.position.x, start.pose.position.y);
    ROS_INFO("Goal: X -> %fm, Y -> %fm", goal.pose.position.x, goal.pose.position.y);

    /*ROS_INFO("rrt_smart_path: %i", rrt_smart_path.size());
	ROS_INFO("rrt_star_path: %i", rrt_star_path.size());
	ROS_INFO("node_beacons: %i", node_beacons.size());
	ROS_INFO("nodes: %i", nodes.size());*/

    auto start_time = std::chrono::high_resolution_clock::now(); // Start of counting time
    Node node_nearest;

    /**************************************************************************************************************
    *********************************************** LOOP INIT *****************************************************
    **************************************************************************************************************/

    while (iterations < MAX_NUM_NODES && !path_found)
    {
	bool path_found = false;
      bool found_next = false;
      while (found_next == false)
      {

        // Depending on the iteration try to find a random point in whole map or inside
        // the near area of node_beacons
        if (iterations % biasing_ratio == 0 && node_beacons.size() != 0)
        {
          p_rand = sampleFreeBeacon(node_beacons, biasing_radius, idx_beacon); // random point near the nodes of
          idx_beacon++;                                                        // actual RRT*-smart path (beacon_nodes)

	  p_new.first = p_rand.first;
	  p_new.second = p_rand.second;

          if (idx_beacon == node_beacons.size())
          {
            idx_beacon = 0;
          }
	  node_nearest = getNearest(nodes, p_new); 
        }
        else
        {
          p_rand = sampleFree(); // random point in the free space
	  node_nearest = getNearest(nodes, p_rand);  // The nearest node of the random point
          p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
        }

        

        if (obstacleFree(node_nearest, p_new.first, p_new.second))
        {
          Node newnode;
          newnode.x = p_new.first;
          newnode.y = p_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_back below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimize Path -> RRT*
          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent
          nodes.push_back(newnode);
          nodes = rewire(nodes, newnode);

          found_next = true;
          iterations = nodes.size();
        }
      }

      // Check if the distance between the goal and the new node is less than
      // the GOAL_RADIUS
      if (pointCircleCollision(p_new.first, p_new.second, goal.pose.position.x, goal.pose.position.y, GOAL_RADIUS))
      {
        // RRT* PATH FOUND

        //auto star = std::chrono::high_resolution_clock::now();

        std::pair<float, std::pair<float, float>> new_point;

        // New goal inside of the goal tolerance
        Node new_goal_node = nodes[nodes.size() - 1];
        Node current_node = new_goal_node;

        current_node = new_goal_node;

        if (current_node.cost + distance(current_node.x, current_node.y, goal.pose.position.x, goal.pose.position.y) < old_cost)
        {
          ROS_INFO("Global Planner: Path found!!! at %i iterations_____________________________", iterations);
          rrt_star_path.clear();
	  aux_path.clear();

          /**************************************************************************************************** 
	  *************** Filling the rrt_star_path array with the best plan found (at the moment) ************
	  ****************************************************************************************************/

          // Last node to Goal Node
          new_point.first = current_node.cost + distance(current_node.x, current_node.y, goal.pose.position.x, goal.pose.position.y);
          new_point.second.first = goal.pose.position.x;
          new_point.second.second = goal.pose.position.y;
          rrt_star_path.insert(rrt_star_path.begin(), new_point);
	  old_cost = new_point.first;

          while (current_node.parent_id != -1)
          {

            new_point.first = current_node.cost;
            new_point.second.first = current_node.x;
            new_point.second.second = current_node.y;
            rrt_star_path.insert(rrt_star_path.begin(), new_point);

            current_node = nodes[current_node.parent_id];
          }

          // First node to Start Node
	  float init_cost = distance(rrt_star_path[0].second.first, rrt_star_path[0].second.second, start.pose.position.x, start.pose.position.y);
          new_point.first = 0.0;
          new_point.second.first = start.pose.position.x;
          new_point.second.second = start.pose.position.y;
          rrt_star_path.insert(rrt_star_path.begin(), new_point);

	  ROS_INFO("NEW Path RRT* Length: %fm", rrt_star_path[rrt_star_path.size() - 1].first);
          ROS_INFO("NEW Path RRT* size: %i nodes", rrt_star_path.size());

	  /**************************************************************************************************** 
	  *********************************** RRT*-Smart Path Creation ****************************************
	  ****************************************************************************************************/

          // Optimization Path (SMART)

          aux_path = visibleNodes(rrt_star_path);	 // Creation of the RRT-Smart Path
	  
          //auto smart = std::chrono::high_resolution_clock::now();

          new_path_cost = aux_path[aux_path.size() - 1].first;

          std::pair<float, float> point;  // aux variable to fill node_beacons array

          if (new_path_cost < old_path_cost && new_path_cost < old_cost)
          {
	    rrt_smart_path.clear();
	    rrt_smart_path = aux_path;
            ROS_INFO("Path RRT-SMART Length: %fm", rrt_smart_path[rrt_smart_path.size() - 1].first);
            ROS_INFO("Path RRT-SMART size: %i nodes", rrt_smart_path.size());
		node_beacons.clear();

            //fill vector node_beacons with the nodes found in path optimization
            for (int i = 0; i < rrt_smart_path.size(); i++)
            {
              point.first = rrt_smart_path[i].second.first;
              point.second = rrt_smart_path[i].second.second;
              node_beacons.insert(node_beacons.end(), point);
              old_path_cost = new_path_cost;
            }
		path_found = true;
          }
        }

      } 

    } // LOOP END

    iterations = 0;

    //Giving path to ROS planner
    if (rrt_smart_path.size() > 0)
    {

      plan.push_back(start);
      ros::Time plan_time = ros::Time::now();

      // convert the points of the RRT*-Smart found to poses
      for (int i = 0; i < rrt_smart_path.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = "map";
        pose.pose.position.x = rrt_smart_path[i].second.first;
        pose.pose.position.y = rrt_smart_path[i].second.second;
	//ROS_INFO("Node(%i): X -> %fm, Y -> %fm",i ,rrt_smart_path[i].second.first, rrt_smart_path[i].second.second);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
      }

      ROS_INFO("_____________________________RESULTS________________________________");
      ROS_INFO("____________________________________________________________________");
      ROS_INFO("FINAL RRT-SMART length: %fm", rrt_smart_path[rrt_smart_path.size() - 1].first);
      ROS_INFO("FINAL RRT-SMART size: %i nodes", rrt_smart_path.size());
      ROS_INFO("LAST RRT* length: %fm", rrt_star_path[rrt_star_path.size() - 1].first);
      ROS_INFO("LAST RRT* size: %i nodes", rrt_star_path.size());
      ROS_INFO("____________________________________________________________________");
      ROS_INFO("____________________________________________________________________");
      //auto time_star = std::chrono::duration_cast<std::chrono::seconds>(star - start_time);
      //auto time_smart = std::chrono::duration_cast<std::chrono::seconds>(smart - start_time);
      //std::chrono::duration<double> time_star = star - start_time;
      //std::chrono::duration<double> time_smart = smart - start_time;
      //ROS_INFO("Time RRT*: %fs", time_star.count());
      //ROS_INFO("Time RRT-SMART: %fs", time_smart.count());
      return true;
    }
    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }
  }

  /**********************************************************************************************************************************************
************************************** Functions **********************************************************************************************
**********************************************************************************************************************************************/

  bool RRTsmartPlannerROS::pointCircleCollision(float x1, float y1, float x2, float y2, float radius)
  {
    float dist = distance(x1, y1, x2, y2);
    if (dist < radius)
      return true;
    else
      return false;
  }

  float RRTsmartPlannerROS::distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2) * (px1 - px2) + (py1 - py2) * (py1 - py2));
    return dist;
  }

  std::pair<float, float> RRTsmartPlannerROS::sampleFree()
  {
    std::pair<float, float> random_point;
    for (int i = 0; i < 10000; i++)
    {
      // generate random x and y coords within map bounds

      std::random_device rd;
      std::mt19937 gen(rd());
      //float map_width = costmap_->getSizeInMetersX();
      //float map_height = costmap_->getSizeInMetersY();

      // Using the clearpath Husky World I know that the dimensions are
      float map_width = 50.0;
      float map_height = 50.0;
      std::uniform_real_distribution<> x(-map_width, map_width);
      std::uniform_real_distribution<> y(-map_height, map_height);

      random_point.first = x(gen);
      random_point.second = y(gen);

      if (!collision(random_point.first, random_point.second))
        return random_point;
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
    }
    return random_point;
  }

  std::pair<float, float> RRTsmartPlannerROS::sampleFreeBeacon(std::vector<std::pair<float, float>> center_beacon, float bias_radius, int beacon)
  {
    std::pair<float, float> random_point_beacon;

    float circle_center_x = center_beacon[beacon].first;
    float circle_center_y = center_beacon[beacon].second;
    double uniform;
    double theta;
    double r;

    for (int i = 0; i < 10000; i++)
    {

      uniform = (double)rand() / RAND_MAX;
      theta = 2 * PI * uniform;
      r = sqrt(uniform);

      random_point_beacon.first = circle_center_x + r * bias_radius * cos(theta);
      random_point_beacon.second = circle_center_y + r * bias_radius * sin(theta);

      if (!collision(random_point_beacon.first, random_point_beacon.second))
        return random_point_beacon;
    }
  }

  void RRTsmartPlannerROS::mapToWorld(int mx, int my, float &wx, float &wy)
  {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  void RRTsmartPlannerROS::worldToMap(float wx, float wy, int &mx, int &my)
  {
    float origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    float resolution = costmap_->getResolution();

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;
  }

  //check if point collides with the obstacle
  bool RRTsmartPlannerROS::collision(float wx, float wy)
  {
    int mx, my;
    worldToMap(wx, wy, mx, my);

    if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
      return true;

    // grid[row][column] = vector[row*WIDTH + column]
    //if (costmap_[my*width + mx] > 0)
    //  return true;

    int unsigned cost = static_cast<int>(costmap_->getCost(mx, my));
    if (cost != 0 && cost != 255) // free && unknown
      return true;

    return false;
  }

  Node RRTsmartPlannerROS::getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand)
  {
    Node node = nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second) < distance(node.x, node.y, p_rand.first, p_rand.second))
        node = nodes[i];
    }

    return node;
  }

  Node RRTsmartPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {

    for (int i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < RADIUS &&
          nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y) &&
          obstacleFree(nodes[i], nn.x, nn.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    newnode.parent_id = nn.node_id;

    return newnode;
  }

  std::vector<Node> RRTsmartPlannerROS::rewire(std::vector<Node> nodes, Node newnode)
  {
    Node node;
    for (int i = 0; i < nodes.size(); i++)
    {
      node = nodes[i];
      if (node != nodes[newnode.parent_id] && distance(node.x, node.y, newnode.x, newnode.y) < RADIUS &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost && obstacleFree(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
      }
    }
    return nodes;
  }

  std::pair<float, float> RRTsmartPlannerROS::steer(float x1, float y1, float x2, float y2)
  {
    std::pair<float, float> p_new;
    float dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max && dist > epsilon_min)
    {
      p_new.first = x1;
      p_new.second = y1;
      return p_new;
    }
    else
    {
      float theta = atan2(y2 - y1, x2 - x1);
      p_new.first = x1 + epsilon_max * cos(theta);
      p_new.second = y1 + epsilon_max * sin(theta);
      return p_new;
    }
  }

  bool RRTsmartPlannerROS::obstacleFree(Node node_nearest, float px, float py)
  {
    int n = 1;
    float theta;

    std::pair<float, float> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    float dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution)
    {
      if (collision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist / resolution));
      float theta;
      for (int i = 0; i < value; i++)
      {
        theta = atan2(node_nearest.y - py, node_nearest.x - px);
        p_n.first = node_nearest.x + n * resolution * cos(theta);
        p_n.second = node_nearest.y + n * resolution * sin(theta);
        if (collision(p_n.first, p_n.second))
          return false;

        n++;
      }
      return true;
    }
  }

  bool RRTsmartPlannerROS::obstacleFreeSmart(float px1, float py1, float px2, float py2)
  {
    int m = 1;
    float theta;

    std::pair<float, float> p_m;

    p_m.first = 0.0;
    p_m.second = 0.0;

    float dist = distance(px1, py1, px2, py2);
   
    int value = int(floor(dist / resolution));
    //float theta;
    theta = atan2(py2 - py1, px2 - px1);

    for (int i = 0; i < value; i++)
    {

	p_m.first = px1 + m * (resolution)*cos(theta);
	p_m.second = py1 + m * (resolution)*sin(theta);

	if (collision(p_m.first, p_m.second))
  	   return false;

	m++;
     }
     return true;
  }

   std::vector<std::pair<float, std::pair<float, float>>> RRTsmartPlannerROS::visibleNodes(std::vector<std::pair<float, std::pair<float, float>>> rrt_star_path)
  {

    int node_beacon = 1;
    int candidate = 1;   

    std::pair<float, std::pair<float, float>> new_node;
    std::vector<std::pair<float, std::pair<float, float>>> rrt_star = rrt_star_path;
    std::vector<std::pair<float, std::pair<float, float>>> rrt_smart;

    //Insert first node of the RRT*-Smart path (start pose)
    new_node.first = 0; //cost new node	 
    new_node.second.first = rrt_star[0].second.first;   // X new node
    new_node.second.second = rrt_star[0].second.second; // Y new node
    rrt_smart.insert(rrt_smart.end(), new_node);
    

    while (node_beacon < rrt_star.size())
    {
	 
	 for (int i = node_beacon; i < rrt_star.size(); i++)
         {
	   //Looking for visible nodes of rrt* path
           if (obstacleFreeSmart(rrt_star[node_beacon].second.first, rrt_star[node_beacon].second.second, rrt_star[i].second.first, rrt_star[i].second.second))
           {
             candidate = i;
           }
         }

         
	 //Filling the smart_path with visible nodes
         new_node.first = distance(rrt_star[candidate].second.first, rrt_star[candidate].second.second, rrt_smart[rrt_smart.size() - 1].second.first, rrt_smart[rrt_smart.size() - 1].second.second) + rrt_smart[rrt_smart.size() - 1].first; //cost new node
         new_node.second.first = rrt_star[candidate].second.first;   // X new node
         new_node.second.second = rrt_star[candidate].second.second; // Y new node
         rrt_smart.insert(rrt_smart.end(), new_node);
        
         
	 // If a visible node (different of current node) is not found
         if (node_beacon = candidate)
         {
            node_beacon++;
         }
         else
         {
           node_beacon = candidate;
         }

    }

    return rrt_smart;
  }

}; // RRTsmart_planner

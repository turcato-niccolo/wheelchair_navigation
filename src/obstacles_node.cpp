#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This node takes charge of
 */

std::string costmap_converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
int costmap_converter_rate = 

int main(int argc, char **argv)
{
    / create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);
    // costmap converter plugin related parameters
    nh.param("/costmap_converter_plugin", costmap_converter_plugin, costmap_converter_plugin);
    nh.param("/costmap_converter_rate",   costmap_converter_rate, costmap_converter_rate);
    nh.param("/costmap_converter_spin_thread", costmap_converter_spin_thread, costmap_converter_spin_thread);



}
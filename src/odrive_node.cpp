#include "odrive_node.hpp"
#include "odrive_axis.hpp"
#include <signal.h>

bool engage_on_startup;
bool disengage_on_shutdown;
std::vector<std::string> axis_names_list;
std::vector<int> axis_can_ids_list;
std::vector<std::string> axis_directions_list;
std::vector<odrive::ODriveAxis *> odrive_axises;

bool intsAreDistinct(std::vector<int> arr) {
    std::unordered_set<int> s;
    for (int i = 0; i < (int)arr.size(); i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

bool stringsAreDistinct(std::vector<std::string> arr) {
    int n = arr.size();
    std::unordered_set<std::string> s;
    for (int i = 0; i < n; i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

void BoolMessagesCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ( msg->data)
    {
        for (int i = 0; i < (int)odrive_axises.size(); i++) {
            odrive_axises[i]->engage();
        }
    }
}

void onShutdown(int sig) {
    (void)sig;
	ROS_INFO("ODrive shutting down");
	if (disengage_on_shutdown) {
    	ROS_INFO("ODrive disengaging motors");
        for (int i = 0; i < (int)odrive_axises.size(); i++) {
            odrive_axises[i]->disengage();
        }
	}
	ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odrive_can_ros_node");
	ros::NodeHandle node("~");
    ROS_INFO("ODrive starting up");
    node.param<bool>("engage_on_startup", engage_on_startup, DEFAULT_ENGAGE_ON_STARTUP);
    node.param<bool>("disengage_on_shutdown", disengage_on_shutdown, DEFAULT_DISENGAGE_ON_SHUTDOWN);
    if (engage_on_startup) {
        ROS_INFO("Will engage axises on startup");
    } else {
        ROS_INFO("Will not engage axises on startup");
    }
    if (!node.hasParam("axis_names")) {
        ROS_ERROR("Can't run without axis_names parameter");
        return -1;
    }
    if (!node.hasParam("axis_can_ids")) {
        ROS_ERROR("Can't run without axis_can_ids parameter");
        return -1;
    }
    if (!node.hasParam("axis_directions")) {
        ROS_ERROR("Can't run without axis_directions parameter");
        return -1;
    }
    node.getParam("axis_names", axis_names_list);
    node.getParam("axis_can_ids", axis_can_ids_list);
    node.getParam("axis_directions", axis_directions_list);

    cl_ctrl_sub = node.subscribe<std_msgs::Bool>("/motor_cl_ctrl", 1, &BoolMessagesCallback);

    if (!(axis_names_list.size() == axis_can_ids_list.size() && axis_can_ids_list.size() ==
        axis_directions_list.size())) {
        ROS_ERROR("axis_names, axis_can_ids and axis_can_directions must be of an equal size");
        return -1;
    }
    if (!stringsAreDistinct(axis_names_list)) {
        ROS_ERROR("axis names must be distinct");
        return -1;
    }
    if (!intsAreDistinct(axis_can_ids_list)) {
        ROS_ERROR("axis CAN ids must be distinct");
        return -1;
    }
    for (int i = 0; i < (int)axis_names_list.size(); i++) {
        ROS_INFO("Adding axis %s with CAN id %d and direction %s", axis_names_list[i].c_str(), 
            axis_can_ids_list[i], axis_directions_list[i].c_str());
        if (axis_names_list[i].length() == 0) {
            ROS_ERROR("axis name can't be empty");
            return -1;
        }
        if (axis_can_ids_list[i] <= 0) {
            ROS_ERROR("axis CAN id must be >0");
            return -1;
        }
        odrive_axises.push_back(new odrive::ODriveAxis(&node, axis_names_list[i], axis_can_ids_list[i], 
            axis_directions_list[i]));
    }
    
    signal(SIGINT, onShutdown);
	ros::spin();
	return 0;
}

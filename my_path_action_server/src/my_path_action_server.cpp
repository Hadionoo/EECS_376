// sin_commander node: 
// node that commands sinusoidal velocities to the minimal controller
// asks the user for an amplitude and a frequency
// publishes to the vel_cmd topic

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <string>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <my_path_action_server/path_msgAction.h>
using namespace std;


class MyPathActionServer {
private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<my_path_action_server::path_msgAction> as_;
    my_path_action_server::path_msgGoal goal_;
    my_path_action_server::path_msgResult result_;
    my_path_action_server::path_msgFeedback feedback_;
    
    //set up variables to check the number of cycles
    //double numCycles;


public:
   
    MyPathActionServer();
    ~MyPathActionServer(void){
    }
    
    void executeCB(const actionlib::SimpleActionServer<my_path_action_server::path_msgAction>::GoalConstPtr& goal);
};    
    //initializing the object as_
    //action server will be known by sine_action
    //this name is what clients use to connect to the server
    MyPathActionServer::MyPathActionServer() :
    as_(nh_, "path_action", boost::bind(&MyPathActionServer::executeCB, this, _1),false)
{

    ROS_INFO("in constructor of MyPathActionServer...");
    as_.start();
}


void MyPathActionServer::executeCB(const actionlib::SimpleActionServer<my_path_action_server::path_msgAction>::GoalConstPtr& goal){
   ROS_INFO("in executeCB");
   geometry_msgs::Twist twist_cmd; // creating variable to be published
   ros::NodeHandle n; // node handle
   ros::Publisher twist_cmd_publisher = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
   twist_cmd = goal->pose;
    twist_cmd_publisher.publish(twist_cmd);
    ROS_INFO("published");
    as_.setSucceeded(result_);

    if (as_.isPreemptRequested()){	
          ROS_WARN("goal cancelled!");
          result_.result_msg = 0;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback

    //feedback_.fdbk = 1; // populate feedback message with current countdown value
 	   //as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal   
     }

    result_.result_msg = 0; //value should be zero, if completed countdown
   }


int main(int argc, char **argv) {
    ros::init(argc, argv, "my_path_action_server"); //this node will be called sin_commander 
    MyPathActionServer as_object;
    ROS_INFO("going into spin");
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
	ros::spinOnce(); 
    }
    return 0;
}



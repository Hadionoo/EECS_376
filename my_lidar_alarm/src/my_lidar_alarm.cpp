// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_DISTANCE_1 = 0.5;
// these values to be set within the laser callback
float ping_dist_in_front_=3.0;
float ping_dist_in_front_1 = 3.0; 
float ping_dist_in_front_2 = 3.0;
float ping_dist_in_front_3 = 3.0;
float ping_dist_in_front_4 = 3.0;
float ping_dist_in_front_5 = 3.0;
float ping_dist_in_front_6 = 3.0;// global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
int ping_index_1 = -1;
int ping_index_2 = -1;
int ping_index_3 = -1;
int ping_index_4 = -1;
int ping_index_5 = -1;
int ping_index_6 = -1;
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;

        angle_max_ = laser_scan.angle_max;

        angle_increment_ = laser_scan.angle_increment;

        range_min_ = laser_scan.range_min;

        range_max_ = laser_scan.range_max;

ROS_INFO("Angle Min: = %d", angle_min_);
ROS_INFO("Angle Max: = %d", angle_max_);
ROS_INFO("Angle Increment: = %d", angle_increment_);
ROS_INFO("Range Min: = %d", range_min_);
ROS_INFO("Range Max: = %d", range_max_);
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ping_index_1 = 220;
ping_index_2 = 362;
ping_index_3 = 382;
ping_index_4 = 322;
ping_index_5 = 312;
ping_index_6 = 276;
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        ROS_INFO("LIDAR setup: ping_index1 = %d",ping_index_1);
        ROS_INFO("LIDAR setup: ping_index2= %d",ping_index_2);
        ROS_INFO("LIDAR setup: ping_index3 = %d",ping_index_3);
        ROS_INFO("LIDAR setup: ping_index4 = %d",ping_index_4);
        ROS_INFO("LIDAR setup: ping_index5 = %d",ping_index_5);
                ROS_INFO("LIDAR setup: ping_index6 = %d",ping_index_6);
    }
   ping_dist_in_front_1 = laser_scan.ranges[ping_index_1]; 
   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ping_dist_in_front_2 = laser_scan.ranges[ping_index_2]; 
   ping_dist_in_front_3 = laser_scan.ranges[ping_index_3]; 
   ping_dist_in_front_4 = laser_scan.ranges[ping_index_4]; 
   ping_dist_in_front_5 = laser_scan.ranges[ping_index_5]; 
   ping_dist_in_front_6 = laser_scan.ranges[ping_index_6]; 
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   ROS_INFO("ping dist in front 1 = %f",ping_dist_in_front_1);
   ROS_INFO("ping dist in front 2 = %f",ping_dist_in_front_2);
   ROS_INFO("ping dist in front 3 = %f",ping_dist_in_front_3);
   ROS_INFO("ping dist in front 4 = %f",ping_dist_in_front_4);
   ROS_INFO("ping dist in front 5 = %f",ping_dist_in_front_5);
   ROS_INFO("ping dist in front 6 = %f",ping_dist_in_front_6);
   if (ping_dist_in_front_<MIN_SAFE_DISTANCE 
|| ping_dist_in_front_1 <MIN_SAFE_DISTANCE_1 || ping_dist_in_front_2 <MIN_SAFE_DISTANCE || ping_dist_in_front_3 < MIN_SAFE_DISTANCE || ping_dist_in_front_4 <MIN_SAFE_DISTANCE || ping_dist_in_front_5 <MIN_SAFE_DISTANCE || ping_dist_in_front_6 <MIN_SAFE_DISTANCE
) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   std_msgs::Float32 lidar_dist_msg_1;
  std_msgs::Float32 lidar_dist_msg_2;
  std_msgs::Float32 lidar_dist_msg_3;
  std_msgs::Float32 lidar_dist_msg_4;
  std_msgs::Float32 lidar_dist_msg_5;
  std_msgs::Float32 lidar_dist_msg_6;
   lidar_dist_msg.data = ping_dist_in_front_;
    lidar_dist_msg_1.data = ping_dist_in_front_1;
    lidar_dist_msg_2.data = ping_dist_in_front_2;
    lidar_dist_msg_3.data = ping_dist_in_front_3;
    lidar_dist_msg_4.data = ping_dist_in_front_4;
    lidar_dist_msg_5.data = ping_dist_in_front_5;
    lidar_dist_msg_6.data = ping_dist_in_front_6;
   lidar_dist_publisher_.publish(lidar_dist_msg);
    lidar_dist_publisher_.publish(lidar_dist_msg_1);   
    lidar_dist_publisher_.publish(lidar_dist_msg_2);   
    lidar_dist_publisher_.publish(lidar_dist_msg_3);   
    lidar_dist_publisher_.publish(lidar_dist_msg_4);   
    lidar_dist_publisher_.publish(lidar_dist_msg_5);
    lidar_dist_publisher_.publish(lidar_dist_msg_6);      

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}


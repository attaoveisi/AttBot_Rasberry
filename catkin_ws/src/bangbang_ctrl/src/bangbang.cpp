#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sstream>

double pos_x,pos_y;
double pos_x_cmd,pos_y_cmd;
double pos_deltax,pos_deltay;
std_msgs::String bangbang_msg;
nav_msgs::Path gPlan;

void cmd_pos_Callback(nav_msgs::Path& msg)
{
    ROS_INFO_STREAM("Received pose: " << msg);
    geometry_msgs::PoseStamped pose;
    int i=0;
    std::vector<geometry_msgs::PoseStamped> data = msg.poses; 
    for(auto it=data.begin(); it!=data.end(); ++it){ 
        //gPlan.poses.header.stamp = gPlan.poses[i].header = it->header;
        pose.pose=it->pose;
        //gPlan.poses.pose = it->pose;
        i++; 
        } 
  // pos_x_cmd = gPlan.poses[1].pose.Pose_.position.x;
  // pos_y_cmd = gPlan.poses[1].pose.Pose_.position.y;
  pos_x_cmd = pose.pose.position.x;
  pos_y_cmd = pose.pose.position.y;
}

void ekf_pos_Callback(geometry_msgs::PoseWithCovarianceStamped& msg)
{
  pos_x = msg.pose.pose.position.x;
  pos_y = msg.pose.pose.position.y;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "bangbang_controller");
  ros::NodeHandle n;
  ros::Publisher bangbang_pub = n.advertise<std_msgs::String>("bangbang", 1);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::Subscriber sub_pose = n.subscribe("robot_pose_ekf/odom_combined", 1, ekf_pos_Callback);
    ros::Subscriber sub_TRJ = n.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, cmd_pos_Callback);
    pos_deltax =  pos_x_cmd-pos_x;
    pos_deltay =  pos_y_cmd-pos_y;

    if(pos_deltax > 0){
        if (abs(pos_deltay) < 0.05){
            bangbang_msg.data = "1";
        }else if(pos_deltay > 0){
            bangbang_msg.data = "2";
        }else{
            bangbang_msg.data = "3";
        }
    }else{
        bangbang_msg.data = "4";
    }
    
    ROS_INFO("%s", bangbang_msg.data.c_str());

    bangbang_pub.publish(bangbang_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

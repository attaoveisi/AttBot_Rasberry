#include "ros/ros.h"
#include "std_msgs/UInt16.h"
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sstream>

double pos_x,pos_y;
double pos_x_cmd,pos_y_cmd;
double pos_deltax,pos_deltay;
std_msgs::UInt16 bangbang_msg;
nav_msgs::Path gPlan;

void cmd_pos_Callback(const nav_msgs::Path& msg)
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
        pos_x_cmd = pose.pose.position.x;
        pos_y_cmd = pose.pose.position.y;
        } 
  // pos_x_cmd = gPlan.poses[1].pose.Pose_.position.x;
  // pos_y_cmd = gPlan.poses[1].pose.Pose_.position.y;
}

void ekf_pos_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
  ROS_INFO("pose_x is:  %f", pos_x);
  ROS_INFO("pose_y is:  %f", pos_y);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "bangbang_controller");
  ros::NodeHandle n;
  ros::Publisher bangbang_pub = n.advertise<std_msgs::UInt16>("/bangbang", 1);
  ros::Rate loop_rate(10);
  ros::Subscriber sub_pose = n.subscribe("/robot_pose_ekf/odom_combined", 1, ekf_pos_Callback);
  ros::Subscriber sub_TRJ = n.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, cmd_pos_Callback);
  ros::spinOnce();

  while (ros::ok()){
    pos_deltax =  (pos_x_cmd-pos_x);
    pos_deltay =  (pos_y_cmd-pos_y);

    if(pos_deltax > 0){
        if (abs(pos_deltay) < 0.05){
            bangbang_msg.data = 1;
        }else if(pos_deltay > 0){
            bangbang_msg.data = 2;
        }else{
            bangbang_msg.data = 3;
        }
    }else{
        bangbang_msg.data = 4;
    }
    
    ROS_INFO("%f", bangbang_msg.data);
    ROS_INFO("delta_x is:  %f", pos_deltax);
    ROS_INFO("delta_y is:  %f", pos_deltay);
    ROS_INFO("pos_x_cmd is:  %f", pos_x_cmd);
    ROS_INFO("pos_y_cmd is:  %f", pos_y_cmd);

    bangbang_pub.publish(bangbang_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

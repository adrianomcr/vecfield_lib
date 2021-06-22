#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "distfield_class.h"

//Parameters of the code
#define pi 3.14159265 //pi
#define freq 30.0 //control loop frequency
#define Ts 1.0/freq

#define v_r 1.0 //robots reference speed

#define d 0.1 //feedback linearization




using namespace std;
using namespace Eigen;


double pos_x = 0;
double pos_y = 0;
double ang_psi = 0;




// Get States Callback
void cb_state(const turtlesim::Pose::ConstPtr &msg){

  pos_x = msg->x;
  pos_y = msg->y;
  ang_psi = msg->theta;
}


Vector2d feedback_linearization(Vector2d vel, double psi){

  Vector2d vw;

  vw(0) = cos(psi)*vel(0) + sin(psi)*vel(1);
  vw(1) = -sin(psi)*vel(0)/d + cos(psi)*vel(1)/d;

  return vw;
}





// Main
int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;


  ros::Subscriber pose_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1, cb_state);
  cout << "\33[94mSubscribing to '/turtle1/pose'\33[0m" << endl;
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  cout << "\33[94mPublishing at to '/turtle1/cmd_vel'\33[0m" << endl;


  ros::Rate loop_rate(freq);


  geometry_msgs::Twist vel_msg;



    // Nonlinear vector field based controller for the drone object
  distfield_class *distfield;
  distfield = new distfield_class(1.0/freq);


  VectorXd pos(3), F(3), vx_wz(2);
  pos << pos_x, pos_y, 1.0;


  cout << "\33[92mStarting turtle controller at "<<freq<<"Hz\33[0m" << endl;

  while (ros::ok()){

    // Read the callbacks
    ros::spinOnce();

    pos(0) = pos_x;
    pos(1) = pos_y;

    F = distfield->compute_field(pos);

    vx_wz = feedback_linearization(F.block(0,0,2,1), ang_psi);


    vel_msg.linear.x = vx_wz(0);
    vel_msg.angular.z = vx_wz(1);


    // Publish rateThrust command
    cmd_pub.publish(vel_msg);

    // Sleep program
    loop_rate.sleep();



  }



}

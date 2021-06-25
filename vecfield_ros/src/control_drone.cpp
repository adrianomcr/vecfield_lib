// Drone Simulation


// ROS stuffs
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>


//Eigen stuff
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//File management
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h> // commands
#include <stdio.h>


// Drone
#include "drone_class.h"
// Acrorate
#include "acrorate_class.h"
// Distance field and drone nonlinear controller
#include "distfield_class.h"


#include "compilation_config.c"


using namespace std;



#define N_STATES 13

#define freq 100.0 //control loop frequency
#define Ts 1.0/freq

//Log files
FILE *states_log;
FILE *controls_log;
FILE *acro_log;
FILE *sensors_log;

double time_log;

Eigen::VectorXd drone_states(N_STATES);

void open_log_files(){

  std::string log_path;
  log_path = "/tmp/";
  

  std::string log_path_str;
  log_path_str = log_path+"states.txt";
  states_log = fopen(log_path_str.c_str(),"w");
  log_path_str = log_path+"controls.txt";
  controls_log = fopen(log_path_str.c_str(),"w");
  log_path_str = log_path+"acro_cmd.txt";
  acro_log = fopen(log_path_str.c_str(),"w");
  log_path_str = log_path+"sensors.txt";
  sensors_log = fopen(log_path_str.c_str(),"w");
}


void close_log_files(){

  fclose(states_log);
  fclose(controls_log);
  fclose(acro_log);
  fclose(sensors_log);

}


void save_to_log(VectorXd x, VectorXd u, VectorXd acro, VectorXd sensors, double time_log){

  fprintf(states_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),x(11),x(12),time_log);
  fflush(states_log);

  fprintf(controls_log,"%f\t%f\t%f\t%f\t%f\n",u(0),u(1),u(2),u(3),time_log);
  fflush(controls_log);

  fprintf(acro_log,"%f\t%f\t%f\t%f\t%f\n",acro(0),acro(1),acro(2),acro(3),time_log);
  fflush(acro_log);

  fprintf(sensors_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",sensors(0),sensors(1),sensors(2),sensors(3),sensors(4),sensors(5),sensors(6),sensors(7),sensors(8),sensors(9),time_log);
  fflush(sensors_log);

}



// Get drone states
void cb_state(const nav_msgs::Odometry::ConstPtr &msg){


drone_states(0) = msg->pose.pose.position.x;
drone_states(1) = msg->pose.pose.position.y;
drone_states(2) = msg->pose.pose.position.z;

drone_states(3) = msg->twist.twist.linear.x;
drone_states(4) = msg->twist.twist.linear.y;
drone_states(5) = msg->twist.twist.linear.z;

drone_states(6) = msg->pose.pose.orientation.w;
drone_states(7) = msg->pose.pose.orientation.x;
drone_states(8) = msg->pose.pose.orientation.y;
drone_states(9) = msg->pose.pose.orientation.z;

drone_states(10) = msg->twist.twist.angular.x;
drone_states(11) = msg->twist.twist.angular.y;
drone_states(12) = msg->twist.twist.angular.z;

}




int main(int argc, char *argv[]){




  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;


  ros::Subscriber states_sub = nh.subscribe<nav_msgs::Odometry>("/drone/gt", 1, cb_state);
  cout << "\33[94mSubscribing to '/drone/gt'\33[0m" << endl;
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Quaternion>("drone/input/rateThrust", 1);
  cout << "\33[94mPublishing at to 'drone/input/rateThrust'\33[0m" << endl;


  ros::Rate loop_rate(freq);



  cout << "\33[32mMain started ...\33[0m" << endl;

  // Parameters object
  std:string param_file = "/home/adrianomcr/ROS_projects/vecfield_ws/src/vecfield_lib/vecfield_ros/config/parameters.yaml";
  cout << "\33[33mCheck the path of the param file on drone_sim.cpp:\33[0m" << endl;
  cout << "\33[33m"<<param_file<<"""\33[0m" << endl;
  // parameters *Params;
  // Params = new parameters(param_file);
  Eigen::VectorXd config_params(2);
  // Read Parameters
  //config_params = Params->get_parameters();
  config_params << 0.52, 9.81;

  cout << "\33[34m\nOpenning logs ...\33[0m" << endl;
  open_log_files();

  // Drone object
  double time_step;
  time_step = TIME_STEP;

  // drone_class *Drone;
  // Drone = new drone_class(time_step, config_params);

  double m = config_params(0);
  double g = config_params(1);

  // double tau_r = m*g;
  // double wx_r = 0.0;
  // double wy_r = 0.0;
  // double wz_r = 0.0;

  // VectorXd omega_r(3);
  // omega_r << wx_r, wy_r, wz_r;


  // Nonlinear vector field based controller for the drone object
  dronefield_class *dronefield;
  dronefield = new dronefield_class(1.0/FREQ_VEC_FIELD_CONTROL, m);

  //Eigen::VectorXd drone_states(N_STATES);
  Eigen::VectorXd acro_reference(4);

  drone_states << 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0, 0.0,0.0,0.0;
  acro_reference << m*g, 0.0,0.0,0.0;

  
  geometry_msgs::Quaternion rateThrust_msg;


  VectorXd zero_vec(12);
  zero_vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  double t, t0;
  t0 = ros::Time::now().toSec();

 while (ros::ok()){

    // Read the callbacks
    ros::spinOnce();


    dronefield->dronefield_control_step(drone_states.block(0,0,10,1));
    acro_reference = dronefield->get_controls();

    cout << "pos: " << drone_states.block(0,0,3,1).transpose() << endl;
    cout << "vel: " << drone_states.block(3,0,3,1).transpose() << endl;
    cout << "quat: " << drone_states.block(6,0,4,1).transpose() << endl;
    cout << "D: " << dronefield->get_distance() << endl;
    cout << "tau: " << acro_reference(0) << endl;
    cout << "omega: " << acro_reference.block(1,0,3,1).transpose() << endl;
    cout << endl;

    rateThrust_msg.w = acro_reference(0);
    rateThrust_msg.x = acro_reference(1);
    rateThrust_msg.y = acro_reference(2);
    rateThrust_msg.z = acro_reference(3);
    

    // Publish rateThrust command
    cmd_pub.publish(rateThrust_msg);

    
    t = ros::Time::now().toSec() - t0;
    save_to_log(drone_states, zero_vec.block(0,0,4,1), acro_reference, zero_vec.block(0,0,10,1), t);

    // Sleep program
    loop_rate.sleep();


  }

  cout << "\33[34m\nClosing logs ...\33[0m" << endl;
  close_log_files();

  cout << "\33[32m\nMain ended\33[0m" << endl;


}

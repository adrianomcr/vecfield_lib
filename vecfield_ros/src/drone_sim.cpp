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

//Log files
FILE *states_log;
FILE *controls_log;
FILE *acro_log;
FILE *sensors_log;

double time_log;



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





int main(int argc, char *argv[]){

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
  cout << "a" << endl;
  config_params << 0.4, 9.81;
  cout << "b" << endl;

  cout << "\33[34m\nOpenning logs ...\33[0m" << endl;
  open_log_files();

  // Drone object
  double time_step;
  time_step = TIME_STEP;

  drone_class *Drone;
  Drone = new drone_class(time_step, config_params);

  double m = config_params(0);
  double g = config_params(1);

  double tau_r = m*g;
  double wx_r = 0.0;
  double wy_r = 0.0;
  double wz_r = 0.0;

  VectorXd omega_r(3);
  omega_r << wx_r, wy_r, wz_r;

  // AcroRate controller object
  acrorate_class *controller;
  controller = new acrorate_class(1.0/FREQ_ACRO_RATE);

  // Nonlinear vector field based controller for the drone object
  dronefield_class *dronefield;
  dronefield = new dronefield_class(1.0/FREQ_VEC_FIELD_CONTROL);

  Eigen::VectorXd drone_states(N_STATES);
  Eigen::VectorXd drone_controls(4);
  Eigen::VectorXd acro_reference(4);
  Eigen::VectorXd sensors(10);
  Eigen::VectorXd omega_filt(3);
  omega_filt << 0,0,0;

  drone_states = Drone->get_states();
  drone_controls = Drone->get_controls();

  //Variables to print the percentage of the code's execution
  double targ_percent = 0;
  double step_percent = 1;

  double T; //seconds
  T = SIMULATION_TIME;
  double t = 0;

  //Counters to control the frequences
  int count_acro_rate = 0;
  int count_vetor_field = 0;


  cout << "\33[34m\nSimulation loop started ...\33[0m" << endl;
  cout << endl << endl;
  for(int k=0; k<(T/time_step); k++){
    

    t = k*time_step;


    if(t >= count_vetor_field*float(1.0/FREQ_VEC_FIELD_CONTROL)){
        count_vetor_field++;
        dronefield->dronefield_control_step(drone_states.block(0,0,10,1));
        acro_reference = dronefield->get_controls();
#ifdef PRINT_TASKS
        cout << "\33[44mexec dronefield_control_step\33[0m"<< endl;
#endif
    }
    


    if(t >= count_acro_rate*float(1.0/FREQ_ACRO_RATE)){
      count_acro_rate++;
      tau_r = acro_reference(0);
      omega_r = acro_reference.block(1,0,3,1);
      controller->set_acrorate_reference(tau_r, omega_r);
#ifdef USE_GT_ON_ACRORATE
      controller->set_omega(drone_states.block(10,0,3,1));
#elif defined(USE_GYRO_ON_ACRORATE)
      controller->set_omega(sensors.block(0,0,3,1));
#else
      cout << "\33[91mAcro rate controller is not running !\33[0m" << endl;
#endif    
      controller->acrorate_control_step();
      drone_controls = controller->get_controls();
#ifdef PRINT_TASKS
      cout << "\33[46mexec acrorate_control_step\33[0m"<< endl;
#endif
    }


    Drone->set_controls(drone_controls);
    Drone->itegration_step();
    drone_states = Drone->get_states();
#ifdef PRINT_TASKS
    cout << "\33[97mexec itegration_step\33[0m"<< endl;
#endif

    Drone->update_sensors();
    sensors = Drone->sensors->get_sensors();

    save_to_log(drone_states, drone_controls, acro_reference, sensors, k*time_step);

#ifdef PRINT_EXECUTION_PERCENTAGE
    if((t/T)*100 > targ_percent){
      cout << "\033[1A\r\33[92mExecution: " << targ_percent << "%\33[0m" << endl;
      targ_percent += step_percent;
    }
#endif

  }

  drone_states = Drone->get_states();
  
  cout << "\33[32mSimulation step success ...\33[0m" << endl;


  cout << "\33[34m\nClosing logs ...\33[0m" << endl;
  close_log_files();

  cout << "\33[32m\nMain ended\33[0m" << endl;


}

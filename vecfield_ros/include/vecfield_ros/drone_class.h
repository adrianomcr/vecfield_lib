#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <string.h>

#include <random> //used to simulate sensors

using namespace Eigen;



class parameters{
    private:
        YAML::Node node;
        std::string file;
        int n_params;

    public:
        parameters(std::string);   // contructor
        ~parameters();             // destructor

        // return parameters - Followin the sequence in yaml file
        VectorXd get_parameters();

};









class sensors_class{

  private:
    VectorXd gyro;
    VectorXd accel;
    VectorXd gps;
    double bar;

    std::random_device rd;

    std::mt19937 gen{rd()};
    

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    sensors_class(double); //Construtor
    ~sensors_class(); //Destructor

    // void update_sensors();

    void update_sensors(VectorXd, VectorXd);

    Vector3d get_gyro();

    Vector3d get_accel();

    Vector3d get_gps();

    double get_bar();

    VectorXd get_sensors();

};






class drone_class{
  private:
    // States of the drone
    VectorXd states;
    // Propeller commands
    VectorXd controls;

    // Time stamp
    double dt;

    // parameters
    double m;
    double g;
    double drag;

    VectorXd current_acc;


  public:
    sensors_class *sensors;



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    drone_class(double, VectorXd); //Construtor
    ~drone_class(); //Destructor

    // Method to perform an integration step
    void itegration_step();

    // Method to set the values of the propeller commands
    void set_controls(VectorXd);

    // Method to set the values of the propeller commands from a acrorate command
    void set_controls_acro(VectorXd);


    //Method to computethe drone full dynamics
	VectorXd continuous_model(VectorXd, VectorXd, double);


	// Get drone states data
	VectorXd get_states();

    // Get drone position data
    VectorXd get_pos();


    // Get drone current acceleration ("applied forces", ths does not include gravity force effect)
    VectorXd get_current_acc();


	// Get drone current controls
	VectorXd get_controls();

    void update_sensors();


    //Useful functions

    // Convert Euler to Quaternion
    VectorXd EulertoQuaternion(VectorXd);
    VectorXd quat2eulerangle(VectorXd);
    MatrixXd angle2rotm(VectorXd);
    VectorXd rotm2angle(MatrixXd);
    MatrixXd quat2rotm(VectorXd);
	VectorXd quat_derivative(VectorXd, VectorXd);
	VectorXd quat_normalize(VectorXd);
    

};











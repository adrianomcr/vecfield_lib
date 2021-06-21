#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <string.h>


using namespace Eigen;





class acrorate_class{
  private:


    //Reference
    double tau_r;
    VectorXd omega_r;

    //Controller related
    VectorXd omega;
    VectorXd int_omega;
    VectorXd dif_omega;
    VectorXd last_error_omega;

    VectorXd u;

    // Time stamp
    double dt;

    // parameters
    double gain_Kp;
    double gain_Ki;
    double gain_Kd;

    

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    acrorate_class(double); //Construtor
    ~acrorate_class(); //Destructor

    // Method to perform a controller loop
    void acrorate_control_step();

    // Method to set the acro rate reference
    void set_acrorate_reference(double, VectorXd);

    // Set drone current angular velocity (on body frame)
    void set_omega(VectorXd);

    // Get drone current controls (velocities of the propellers)
    VectorXd get_reference();

    // Get drone current controls (velocities of the propellers)
    VectorXd get_controls();

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

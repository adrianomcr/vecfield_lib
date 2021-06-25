#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <string.h>


using namespace Eigen;





class distfield_class{
  protected:




    //Controller related
    VectorXd int_omega;
    VectorXd dif_omega;
    VectorXd last_error_omega;

    VectorXd F; //Value of the vector field

    // Time stamp
    double dt;

    // parameters
    double gain_Kf;
    double vr;

    double Dist;

    

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // distfield_class(); //Construtor
    distfield_class(double); //Construtor
    ~distfield_class(); //Destructor

    // Method to perform a controller loop
    void distfield_control_step(VectorXd);

    // Method to perform a controller loop
    VectorXd compute_field(VectorXd);

    Matrix3d compute_field_Jacobian(VectorXd p);

    Vector3d sample_curve(double);

    double aurea(double, double, VectorXd);

    // Get drone current vector field
    VectorXd get_field();

    // Get the last computed distance to the curve
    double get_distance();

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








class dronefield_class : public distfield_class{



    double gain_Kv;
    double gain_Kw;

    //Controls to be computed
    double tau;
    Vector3d omega;

    //Drone's mass
    double m;

    //Reference
    double tau_r;
    VectorXd omega_r;

    

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    dronefield_class(double, double); //Construtor
    ~dronefield_class(); //Destructor

    // Method to perform a controller loop
    void dronefield_control_step(VectorXd);

    Matrix3d get_orientation_ref(Vector3d, double);

    Vector3d get_acc_ref(Vector3d, Vector3d);

    // Get drone current controls (velocities of the propellers)
    VectorXd get_controls();

};

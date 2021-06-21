// Drone Class for Dynamics simulation (Lib)
/*
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
Adriano M. C. Rezende <adrianomcr18@gmail.com>
*/

#include "drone_class.h"



#define PI 3.1415926535

using namespace std;
using namespace Eigen;

// States
// x, y, z
// vx, vy, vz, (world)
// qw, qx, qy, qz
// wx, wy, wz (body)

#define N_STATES 13



// #define g 9.81
// #define m 1.5


#define Kp 2.9842e-5

#define Ix 3.8278e-3
#define Iy 3.8278e-3
#define Iz 7.1345e-3

// J = [Ix 0 0;
//      0 Iy 0;
//      0 0 Iz];
#define L 0.205
#define d L/1.4142
#define Kd 3.232e-7

#define Jr 2.8385e-5
// #define Jr 0

#define Kdax 5.567e-4
#define Kday 5.567e-4
#define Kdaz 6.354e-4
// Kda = [Kdax 0 0;
       // 0 Kdax 0;
       // 0 0 Kdax];


#define u0 181.3095; //propeller rotation for hovering
#define u_max 1.4142*u0;
#define u_min 0.1*u0;




// Constructor
drone_class::drone_class(double time_step, VectorXd parameters){

  VectorXd states_init(N_STATES);

  //Intial condition - 
  states_init.block(0,0,3,1) << 0,0,0;
  states_init.block(3,0,3,1) << 0,0,0;
  states_init.block(6,0,4,1) << 1,0,0,0;
  states_init.block(10,0,3,1) << 0,0,0;

  //Intial condition - start with nonzero orientation and speed
  // states_init.block(0,0,3,1) << 2,5,-3;
  // states_init.block(3,0,3,1) << 0,0.5,0;
  // states_init.block(6,0,4,1) << 1.0,0.5,0.5,0.5;
  // states_init.block(10,0,3,1) << -1,1,2;

  states_init.block(6,0,4,1) = quat_normalize(states_init.block(6,0,4,1));

  // drone mass
  m = parameters(0);
  // gravity acceleration
  g = parameters(1);
  // drag
  drag = parameters(2);

  // MatrixXd Kda(3,3);
  // Kda << Kdax,0,0,     0,Kday,0,      0,0,Kdaz;

  VectorXd controls_init(4);
  controls_init << m*g,0,0,0;

  states = states_init;
  controls = controls_init;

  VectorXd init_acc(3);
  init_acc << 0.0, 0.0, g;
  current_acc = init_acc;

  dt = time_step;


  sensors = new sensors_class(g);


}




// Method to set the values of the propeller commands
void drone_class::set_controls(VectorXd u){

    controls = u;

}



// Drone model in continuos time
VectorXd drone_class::continuous_model(VectorXd x, VectorXd u, double dt){


    //Create output vector
    VectorXd f(N_STATES);
    //Auxiliary vectors
    VectorXd p(3),v(3),q(4),w(3);
    VectorXd p_dot(3),v_dot(3),q_dot(4),w_dot(3);
    MatrixXd R_bw(3,3);
    VectorXd z_hat(3);
    double tau;
    VectorXd omega(3);

    z_hat << 0,0,1;

    p = x.block(0,0,3,1);
    v = x.block(3,0,3,1);
    q = x.block(6,0,4,1);
    w = x.block(10,0,3,1);

    R_bw = quat2rotm(q);

    VectorXd F(4); //vector of forces
    F = Kp*u.cwiseProduct(u);


    VectorXd T(3); //vector of torques
    T << d*(F(0)-F(1)-F(2)+F(3)),
         d*(-F(0)-F(1)+F(2)+F(3)),
         Kd*(u(0)*u(0)-u(1)*u(1)+u(2)*u(2)-u(3)*u(3)); //simple quadratic propulsion model

    VectorXd Td(3); //rotational drag
    MatrixXd Kda(3,3);
    Kda << Kdax,0,0,     0,Kday,0,      0,0,Kdaz;
    Td = Kda*w;

    VectorXd Tg(3); //torque due to giroscopic effect
    Vector3d v1, v2;
    int sig = 1;
    Tg << 0,0,0;
    for (int i; i<4; i++){
        v1 = w;
        v2 << 0, 0, sig*u(i);
        sig = -sig;
        v2 = Jr*v2;
        Tg = Tg + v1.cross(v2);
    }

    tau = F.sum();

    MatrixXd J(3,3);
    J << Ix,0,0,    0,Iy,0,     0,0,Iz;

    v1 = w;
    v2 = J*w;

    p_dot = v;
    v_dot = R_bw*z_hat*tau/m - g*z_hat; // add drag
    q_dot = quat_derivative(q, R_bw*w);
    w_dot << J.inverse()*(-v1.cross(v2) + T - Td - Tg); //temp // include model

    current_acc = z_hat*tau/m; // add drag

    f.block(0,0,3,1) = p_dot;
    f.block(3,0,3,1) = v_dot;
    f.block(6,0,4,1) = q_dot;
    f.block(10,0,3,1) = w_dot;

    return f;

}




// Integration Step
void drone_class::itegration_step(){

    // Next state function
    VectorXd f(N_STATES);

    //Compute the next state
    f = drone_class::continuous_model(states, controls, dt);    

    // Euler integration
    states = states + f*dt;
    // Renormalize quaternion
    states.block(6,0,4,1) = quat_normalize(states.block(6,0,4,1));
}






VectorXd drone_class::get_states(){

    Eigen::VectorXd states_return(N_STATES);
    states_return = states;

    return states_return;
}

VectorXd drone_class::get_pos(){

    Eigen::VectorXd states_return(3);
    states_return = states.block(0,0,3,1);

    return states_return;
}


VectorXd drone_class::get_controls(){

    Eigen::VectorXd controls_return(4);
    controls_return = controls;

    return controls_return;
}


VectorXd drone_class::get_current_acc(){

    return current_acc;
}


void drone_class::update_sensors(){

    sensors->update_sensors(states, current_acc);
}


// Destructor
drone_class::~drone_class(){
}




//----- ------------ -----//
//----- SENSOR CLASS -----//
//----- ------------ -----//



// Constructor
sensors_class::sensors_class(double g){

    Vector3d zero_vec;
    zero_vec << 0.0, 0.0, 0.0;

    gyro = zero_vec;
    accel = zero_vec; accel(2) = accel(2) + g;
    gps = zero_vec;
    bar = 0;


    
}



void sensors_class::update_sensors(VectorXd x, VectorXd current_acc){

    
    normal_distribution<float> d_gyro(0, 0.05); // [mean, variance]
    normal_distribution<float> d_accel(0, 0.5);
    normal_distribution<float> d_gps_xy(0, 0.2);
    normal_distribution<float> d_gps_z(0, 0.5);
    normal_distribution<float> d_bar(0, 0.1);


    gyro = x.block(10,0,3,1);
    gyro(0) += d_gyro(gen);
    gyro(1) += d_gyro(gen);
    gyro(2) += d_gyro(gen);

    accel = current_acc;
    accel(0) += d_accel(gen);
    accel(1) += d_accel(gen);
    accel(2) += d_accel(gen);

    gps = x.block(0,0,3,1);
    gps(0) += d_gps_xy(gen);
    gps(1) += d_gps_xy(gen);
    gps(2) += d_gps_z(gen);

    bar = x(2);
    bar += d_bar(gen);


}







Vector3d sensors_class::get_gyro(){
    return gyro;
}

Vector3d sensors_class::get_accel(){
    return accel;
}

Vector3d sensors_class::get_gps(){
    return gps;
}

double sensors_class::get_bar(){
    return bar;
}

VectorXd sensors_class::get_sensors(){

    VectorXd s(10);
    s << gyro, accel, gps, bar;
    return s;
}



// Destructor
sensors_class::~sensors_class(){
}



//----- ----------------- -----//
//----- AUXILIARY METHODS -----//
//----- ----------------- -----//






//VectorXd drone_class::EulertoQuaternion( double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
VectorXd drone_class::EulertoQuaternion( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
{

    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double w = cy * cp * cr + sy * sp * sr;
    double x = cy * cp * sr - sy * sp * cr;
    double y = sy * cp * sr + cy * sp * cr;
    double z = sy * cp * cr - cy * sp * sr;

    VectorXd q(4);
    q << w,x,y,z;
    return q;
}


MatrixXd drone_class::angle2rotm( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
{

    MatrixXd Rot(3,3);
    double phi = rpy(0);
    double theta = rpy(1);
    double psi = rpy(2);
    // Get rotation matrix
    Rot << (cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)),
            (cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)),
            (-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta));

    return Rot;
}

VectorXd drone_class::rotm2angle( MatrixXd Rot)
{
    VectorXd rpy(3);
    Matrix3d Rot2;
    //cout << "A2" << endl;
    Rot2 = Rot;
    //cout << "B2" << endl;
    Quaterniond quat1(Rot2);
    //cout << "C2" << endl;
    VectorXd quat2(4);
    //cout << "D2" << endl;
    quat2 << quat1.w(), quat1.x(), quat1.y(), quat1.z();
    //cout << "E2" << endl;
    rpy = quat2eulerangle(quat2);
    //cout << "F2" << endl;
    return rpy;
}


// Unit Quaternion to Euler angle
VectorXd drone_class::quat2eulerangle(VectorXd q){
    // w x y z

    VectorXd angle(3,1);
    angle.setZero();

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    angle[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q[0] * q[2] - q[3] * q[1]);
    if (fabs(sinp) >= 1)
        angle[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angle[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    angle[2] = atan2(siny_cosp, cosy_cosp);

    return angle;
}






// Unit quaternion to rotation matrix
MatrixXd drone_class::quat2rotm(VectorXd q){
    // w x y z

    MatrixXd Rot(3,3);

    double qw, qx, qy, qz;
    qw = q(0);
    qx = q(1);
    qy = q(2);
    qz = q(3);


    Rot << 1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw),
            2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw),
            2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy); //this was checked on matlab
           

    return Rot;
}





// Unit quaternion to rotation matrix
VectorXd drone_class::quat_derivative(VectorXd q, VectorXd w){
    // w x y z
    //velocity w in the world frame

    double qw, qx, qy, qz;
    qw = q(0);
    qx = q(1);
    qy = q(2);
    qz = q(3);
    VectorXd f(4);

    f(0) = -0.5*(w(0)*qx+w(1)*qy+w(2)*qz);
    f(1) = 0.5*(w(0)*qw+w(1)*qz-w(2)*qy);
    f(2) = 0.5*(w(1)*qw+w(2)*qx-w(0)*qz);
    f(3) = 0.5*(w(2)*qw+w(0)*qy-w(1)*qx);

    //cout << "f: " << f.transpose() << endl;

    return f;
}


// Normalize the unit quaternion
VectorXd drone_class::quat_normalize(VectorXd q){

    VectorXd q2(4);

	q2 = q/q.norm();

    return q2;
}











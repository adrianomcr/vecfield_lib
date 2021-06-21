#include "acrorate_class.h"



#define PI 3.1415926535

using namespace std;
using namespace Eigen;


#define g 9.81
#define m 1.5

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
// #define u_max 1.4142*u0;
// #define u_min 0.1*u0;
#define u_max 300.0;
#define u_min 20.0;



// #define ALPHA_OMEGA 0.1
// #define ALPHA_U 0.05
#define ALPHA_OMEGA 0.1
#define ALPHA_U 1



#define MAX_ACC_ANG 4000.0


// Constructor
acrorate_class::acrorate_class(double time_step){

	VectorXd zero_vec(3);
	zero_vec << 0.0, 0.0, 0.0;
	VectorXd ones_vec(4);
	ones_vec << 1.0, 1.0, 1.0, 1.0;
    

	tau_r = m*g;
	omega_r = zero_vec;

	omega = zero_vec;
	int_omega = zero_vec;
	dif_omega = zero_vec;
	last_error_omega = zero_vec;
	u = ones_vec;
	u = u*u0;

	gain_Kp = 30;
	gain_Ki = 8;
	gain_Kd = 5.0*0;

	dt = time_step;
}


void acrorate_class::set_acrorate_reference(double new_tau_r, VectorXd new_omega_r){

	tau_r = new_tau_r;
	omega_r = new_omega_r;
}


void acrorate_class::set_omega(VectorXd new_omega){

	omega = ALPHA_OMEGA*new_omega + (1.0-ALPHA_OMEGA)*omega;
}


//Acro rate controller
void acrorate_class::acrorate_control_step(){

	// double ALPHA = 0.01;
	double ALPHA = 1;

    VectorXd error_omega(3);
    error_omega = omega_r-omega;
	int_omega = int_omega + error_omega*dt;
	dif_omega = (1-ALPHA)*dif_omega + ALPHA*(error_omega - last_error_omega)/dt;
	last_error_omega = error_omega;
	MatrixXd J(3,3);
	J << Ix,0,0,    0,Iy,0,     0,0,Iz;
	Vector3d v1,v2;
	v1 = omega;
	v2 = J*omega;


	VectorXd T_desired(3);
	T_desired = v1.cross(v2) + J*(gain_Kp*error_omega + gain_Ki*int_omega + gain_Kd*dif_omega);
	VectorXd TF_desired(4);
	TF_desired << tau_r, T_desired;
	MatrixXd M(4,4);
	M << Kp, Kp, Kp, Kp,	d*Kp, -d*Kp, -d*Kp, d*Kp,	-d*Kp, -d*Kp, d*Kp, d*Kp,	Kd, -Kd, Kd, -Kd;
    VectorXd u_square(4);
	u_square = M.inverse()*TF_desired;
	for (int i=0; i<4; i++){
		if(u_square[i] < 20*20){
			u_square[i] = 20*20; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! check u_min
		}
		if(u_square[i] > 300*300){
			u_square[i] = 300*300; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! check u_max
		}
	}
	VectorXd u_cmd(4);
	u_cmd = u_square.cwiseSqrt();

	for (int i=0; i<4; i++){
		if(u_cmd(i) > u(i)+MAX_ACC_ANG*dt){
			u(i) = u(i)+MAX_ACC_ANG*dt;
		}
		else if(u_cmd(i) < u(i)-MAX_ACC_ANG*dt){
			u(i) = u(i)-MAX_ACC_ANG*dt;
		}
		else{
			u(i) = u_cmd(i);
		}
	}
	
}



//Get current reference for thrust and angular rates
VectorXd acrorate_class::get_reference(){

    VectorXd reference_return(4);

    reference_return << tau_r, omega_r;

    return reference_return;
}

//Get current control signals for the propellers
VectorXd acrorate_class::get_controls(){

    VectorXd controls_return(4);
    controls_return = u;

    return controls_return;
}



// Destructor
acrorate_class::~acrorate_class(){
}













//----- ----------------- -----//
//----- AUXILIARY METHODS -----//
//----- ----------------- -----//






//VectorXd acrorate_class::EulertoQuaternion( double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
VectorXd acrorate_class::EulertoQuaternion( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
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


MatrixXd acrorate_class::angle2rotm( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
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

VectorXd acrorate_class::rotm2angle( MatrixXd Rot)
{
    VectorXd rpy(3);
    Matrix3d Rot2;

    Rot2 = Rot;
    Quaterniond quat1(Rot2);
    VectorXd quat2(4);
    quat2 << quat1.w(), quat1.x(), quat1.y(), quat1.z();
    rpy = quat2eulerangle(quat2);

    return rpy;
}


// Unit Quaternion to Euler angle
VectorXd acrorate_class::quat2eulerangle(VectorXd q){
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
MatrixXd acrorate_class::quat2rotm(VectorXd q){
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
VectorXd acrorate_class::quat_derivative(VectorXd q, VectorXd w){
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
VectorXd acrorate_class::quat_normalize(VectorXd q){

    VectorXd q2(4);

	q2 = q/q.norm();

    return q2;
}






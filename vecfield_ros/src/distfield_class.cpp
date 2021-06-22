#include "distfield_class.h"



#define PI 3.1415926535
#define GOLD 0.6180339887

using namespace std;
using namespace Eigen;


#define g 9.81
#define m 0.4

#define n_points 30

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



// Constructor
distfield_class::distfield_class(double time_step){


    VectorXd zero_vec(3);
    zero_vec << 0.0, 0.0, 0.0;

    tau_r = m*g;
    omega_r = zero_vec;

    int_omega = zero_vec;
    dif_omega = zero_vec;
    last_error_omega = zero_vec;
    F = zero_vec;

    //Parameters
    gain_Kf = 0.8; //conergence gain of the field
    vr = 1.5; //reference velocity

    dt = time_step;

}



// Function to sample the curve
Vector3d distfield_class::sample_curve(double s){

  Vector3d C;

  // C << 3*cos(s)+5, 2*sin(s)+5, 1; //ellipse
  C << 3*pow(cos(s)*cos(s)*cos(s)*cos(s)+sin(s)*sin(s)*sin(s)*sin(s),-0.25)*cos(s)+5, 2*pow(cos(s)*cos(s)*cos(s)*cos(s)+sin(s)*sin(s)*sin(s)*sin(s),-0.25)*sin(s)+5, 1; //square
  // C << 3*(1+cos(6*s)/6.0)*cos(s)+5, 2*(1+cos(6*s)/6.0)*sin(s)+5, 1; //amoeba 
  // C << 2*(sin(s)+2*sin(2*s)), 2*(cos(s)-2*cos(2*s)), 2*(-sin(3*s)+1) + 1; //knot
  // C << 6*cos(s), 1.5*sin(2*s), 1; //8 - should not work properly for now

  return C;

}




// Golden search optimization method
double distfield_class::aurea(double al, double bl, VectorXd x){

    Vector3d dist_vec, ra, rb;
    double func_a, func_b;
    double s_a, s_b;
    double s_star;

    s_a = bl - GOLD*(bl-al);
    s_b = al + GOLD*(bl-al);

    ra = sample_curve(s_a);
    dist_vec = x-ra;
    func_a = dist_vec.norm();

    rb = sample_curve(s_b);
    dist_vec = x-rb;
    func_b = dist_vec.norm();

    int k=0;
    while(bl-al > 0.000001){

        k = k+1;
        if (k==200) {break;}

        if(func_a > func_b){
            al = s_a;
            s_a = s_b;
            s_b = al + GOLD*(bl-al);
        }
        else{
            bl = s_b;
            s_b = s_a;
            s_a = bl - GOLD*(bl-al);
        }

        ra = sample_curve(s_a);
        dist_vec = x-ra;
        func_a = dist_vec.norm();

        rb = sample_curve(s_b);
        dist_vec = x-rb;
        func_b = dist_vec.norm();
    }
    s_star = (al+bl)/2;


    return s_star;
}


#define CURVE_A 1 //limit cycle
// #define CURVE_B 1 //stable  focus
// #define CURVE_C 1 //stable node


VectorXd distfield_class::compute_field(VectorXd p){



    VectorXd f(3);
    double G, H;

#ifdef CURVE_A
    double s[n_points];
    double ds;
    MatrixXd C(3,n_points);
    double s_star;
    Vector3d r_star, r_star_M, r_star_m;
    double delta_s = 0.000001;

    // Create a vector of parameters
    ds = 2*PI/n_points;   //parametrix that value of 2*PI !!!!!!!!!!!!!!!!!
    s[0] = 0;
    for (int k=1; k<n_points; k++){
        s[k] = s[k-1]+ds;
    }

    //Evaluate the curve thrugh the parameters
    for (int k=0; k<n_points; k++){
        C.block(0,k,3,1) = sample_curve(s[k]);
    }


    // Compute the closest point in the curve and the distance to it
    int s_cl = 0;
    double D = (C.block(0,s_cl,3,1)-p).norm();
    for (int i = 1; i<n_points; i++){
        if ((C.block(0,i,3,1)-p).norm() < D){
            D = (C.block(0,i,3,1)-p).norm();
            s_cl = i; //index of the closest point
        }
    }
    // Use the golden search method to refine the solution
    s_star = aurea(s[s_cl]-1.0*ds, s[s_cl]+1.0*ds, p);
    // Compute the curve at the closest point
    r_star = sample_curve(s_star);
    // Compute the distance vector and the distance function
    Vector3d Dv;
    Dv = p-r_star;
    D = Dv.norm();
    // Compute the gradient of the distance function
    Vector3d grad_D;
    grad_D = Dv/(D+0.000001);

    // Evaluate the curve at neighborhood
    r_star_M = sample_curve(s_star+delta_s);
    r_star_m = sample_curve(s_star-delta_s);

    //Compute the Tangent vector
    Vector3d T;
    T = (r_star_M-r_star_m)/(2*delta_s);
    T = T/(T.norm()+0.0000001);

    // Compute the gain functions of the vector field
    G = (2/PI)*atan(gain_Kf*D);
    H = sqrt(1-G*G);


    // Compute the convergent vector
    Vector3d Psi_g;
    Psi_g = -G*grad_D;
    // Compute the tangent vector
    Vector3d Psi_h;
    Psi_h = H*T;

    double eta = vr;

    // Compute the scalled field
    Vector3d Phi;
    Phi = eta*(Psi_g+Psi_h);

    f = Phi;

#endif

    //---------- ESTABLE FOCUS ----------
#ifdef CURVE_B
    Matrix3d A;
    A << -0.5, 0.9, 0,    -0.9, -0.5, 0,    0, 0, -1;

    f = A*p;
#endif
    //---------- ------------- ----------


    //---------- ESTABLE POINT ----------
#ifdef CURVE_C
    G = (2.0/PI)*atan(gain_Kf*p.norm());

    f = -vr*G*p/(p.norm()+0.000001);
#endif
    //---------- ------------- ----------

    return f;

}


//Compute the Jacobian matrix of the vector field
Matrix3d distfield_class::compute_field_Jacobian(VectorXd p){

    Matrix3d J;
    Vector3d f0, fx, fy, fz;
    Vector3d px, py, pz;
    double delta = 0.0001;

    px = p;
    py = p;
    pz = p;

    px(0) += delta;
    py(1) += delta;
    pz(2) += delta;

    f0 = compute_field(p);
    fx = compute_field(px);
    fy = compute_field(py);
    fz = compute_field(pz);

    J << (fx-f0)/delta, (fy-f0)/delta, (fz-f0)/delta;

    return J;

}

void distfield_class::distfield_control_step(VectorXd p){

    F = compute_field(p);

}


VectorXd distfield_class::get_field(){

    VectorXd controls_return(3);
    controls_return = F;

    return controls_return;
}


// Destructor
distfield_class::~distfield_class(){
}













//----- ------------------------------------ -----//
//----- Methods specific to dronefield_class -----//
//----- ------------------------------------ -----//



// Constructor
dronefield_class::dronefield_class(double time_step): distfield_class(time_step){

    VectorXd zero_vec(3);
    zero_vec << 0.0, 0.0, 0.0;

    omega = zero_vec;
    tau = 0.4*9.81;

    gain_Kv = 0.8;
    gain_Kw = 2.0;

}



// Compute a reference oientation matrix
Matrix3d dronefield_class::get_orientation_ref(Vector3d a_r, double psi_r){

  Vector3d x_r, y_r, z_r, w_psi;
  Matrix3d Rr;
  z_r = a_r/(a_r.norm() + 0.00000000001);
  w_psi << cos(psi_r), sin(psi_r), 0;
  x_r = (w_psi - w_psi.dot(z_r)*z_r)/((w_psi - w_psi.dot(z_r)*z_r).norm() + 0.00000000001);
  y_r = z_r.cross(x_r);
  Rr << x_r, y_r, z_r;

  return Rr;
}



// Compute a reference acceleration
Vector3d dronefield_class::get_acc_ref(Vector3d p, Vector3d v){

  Vector3d a_r;

  Vector3d f;
  Matrix3d J;

  f = compute_field(p);
  J = compute_field_Jacobian(p);


  a_r = J*v + gain_Kv*(f-v);
  a_r(2) += g;

  return a_r;
}





void dronefield_class::dronefield_control_step(VectorXd states){

  Vector3d pos;
  Vector3d vel;
  VectorXd quat(4);
  Matrix3d R;

  pos = states.block(0,0,3,1);
  vel = states.block(3,0,3,1);
  quat = states.block(6,0,4,1);
  R = quat2rotm(quat);

  //Get the z axis in the body and in the world frame
  Vector3d z_b, z_hat;
  z_b = R.block(0,2,3,1);
  z_hat << 0, 0, 1;

  double psi_r, psi_r_M, psi_r_m;
  psi_r = -PI/3*0;
  psi_r_M = -PI/3*0;
  psi_r_m = -PI/3*0;

  // Computation of reference orientation
  Vector3d a_r, a_r_M, a_r_m;
  a_r = get_acc_ref(pos,vel);
  Matrix3d Rr, Rr_M, Rr_m;
  Rr =  get_orientation_ref(a_r, psi_r);

  // Control signal for thrust
  tau = m*a_r.dot(z_b);
  if (tau_r < 0.1){
    tau_r = 0.1;
  }
  
  // Computation of the derivative of the matrix Rr
  double delta_t = 0.01;
  Vector3d pos_M, vel_M, pos_m, vel_m;
  pos_M = pos + vel*delta_t;
  vel_M = vel + (z_b*tau_r/m -g*z_hat)*delta_t;    //add drag """""""""""
  a_r_M = get_acc_ref(pos_M,vel_M);
  Rr_M =  get_orientation_ref(a_r_M, psi_r_M);

  pos_m = pos - vel*delta_t;
  vel_m = vel - (z_b*tau_r/m -g*z_hat)*delta_t;    //add drag """""""""""
  a_r_m = get_acc_ref(pos_m,vel_m);
  Rr_m =  get_orientation_ref(a_r_m, psi_r_m);

  Matrix3d Re, Rr_dot, S_w;

  // Orientation error
  Re = R.transpose()*Rr;

  // Time derivative of Rr
  Rr_dot = (Rr_M-Rr_m)/(2*delta_t);

  S_w = R.transpose()*Rr_dot;
  S_w = S_w*(Re.transpose());

  // Computation of the feedforward angular velocity
  Vector3d omega_d;
  omega_d << S_w(2,1)-S_w(1,2), S_w(0,2)-S_w(2,0), S_w(1,0)-S_w(0,1);
  omega_d = omega_d/2.0;

  // // Computation of the axis angle representation
  Vector3d axis;

  AngleAxisd angle_axis(Re);
  double alpha = angle_axis.angle();
  axis = angle_axis.axis();

  omega = omega_d + gain_Kw*sin(alpha)*axis;
  // omega = gain_Kw*sin(alpha)*axis;

}



VectorXd dronefield_class::get_controls(){

    VectorXd controls_return(4);
    controls_return << tau, omega;
    // controls_return << tau, omega(0), omega(1), omega(2);

    return controls_return;
}



// Destructor
dronefield_class::~dronefield_class(){
}


















//----- ----------------- -----//
//----- AUXILIARY METHODS -----//
//----- ----------------- -----//






//VectorXd distfield_class::EulertoQuaternion( double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
VectorXd distfield_class::EulertoQuaternion( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
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


MatrixXd distfield_class::angle2rotm( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
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

VectorXd distfield_class::rotm2angle( MatrixXd Rot)
{
    VectorXd rpy(3);
    Matrix3d Rot2;

    Rot2 = Rot;
    Quaterniond quat1(Rot2);
    VectorXd quat2(4);
    quat2 << quat1.w(), quat1.x(), quat1.y(), quat1.z();;
    rpy = quat2eulerangle(quat2);

    return rpy;
}


// Unit Quaternion to Euler angle
VectorXd distfield_class::quat2eulerangle(VectorXd q){
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
MatrixXd distfield_class::quat2rotm(VectorXd q){
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
VectorXd distfield_class::quat_derivative(VectorXd q, VectorXd w){
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
VectorXd distfield_class::quat_normalize(VectorXd q){

    VectorXd q2(4);

	q2 = q/q.norm();

    return q2;
}






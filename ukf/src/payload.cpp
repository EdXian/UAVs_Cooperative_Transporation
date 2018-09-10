#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/VFR_HUD.h>
#include <UKF/output.h>
#include <std_msgs/Float64.h>
#include <string>
using namespace std;
geometry_msgs::PoseWithCovarianceStamped svo_pose;
geometry_msgs::PoseStamped mocap_pose;
sensor_msgs::Imu imu_data;
nav_msgs::Odometry filterd;
mavros_msgs::VFR_HUD vfr_data;
UKF::output output_data;
geometry_msgs::PoseStamped measure_data, output_vc, output_omegac, output_leader;

void svo_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  svo_pose = *msg;
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_pose = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;//test
}

void vfr_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg){
  vfr_data = *msg;//test
}

void measure_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  measure_data = *msg;
}
//convert body frame to inertial frame
Eigen::MatrixXd R_B_I;

void output_cb(const UKF::output::ConstPtr &msg){
  output_data = *msg;
  double roll = output_data.theta.x;
  double yaw = output_data.theta.z;
  double pitch = output_data.theta.y;

  double cr= cos(roll) , cy=cos(yaw) , cp = cos(pitch);
  double sr = sin(roll) , sy =sin(yaw) , sp = sin(pitch);
  R_B_I.setZero(3,3);
  R_B_I << cy * cp , (cy * sp * sr - sy * cr) ,(cy * sp * cr + sy * sr) ,

        sy * cp  , (sy * sp * sr + cy * cr) ,(sy * sp * cr - cy * sr) ,
        -sp ,  cp * sr  , cp * cr;


}
struct Measurement
{
  // The measurement and its associated covariance
  std::vector<float> measurement_;
  Eigen::MatrixXd covariance_;
  int updatesize;
  double mahalanobisThresh_;


};
Measurement measurement;

// Global variable
const int STATE_SIZE = 28;
Eigen::VectorXd state_(STATE_SIZE); //x
Eigen::MatrixXd weightedCovarSqrt_(STATE_SIZE,STATE_SIZE); // square root of (L+lamda)*P_k-1
Eigen::MatrixXd estimateErrorCovariance_(STATE_SIZE,STATE_SIZE); // P_k-1
Eigen::VectorXd process_noise(STATE_SIZE);
std::vector<Eigen::VectorXd> sigmaPoints_, sigmaPoints_a;
std::vector<double> stateWeights_;
std::vector<double> covarWeights_;
double lambda_;
bool uncorrected_;
int flag;
int flag2;
int flag3;
float payload_pitch, payload_yaw, payload_roll;
float thrust;
float a_g;
float imu_ax_bias;
float imu_ay_bias;
/*test variable*/




enum StateMembers
{
  x_c = 0,
  y_c,
  z_c,
  Vx_c,
  Vy_c,
  Vz_c,
  pitch_c,
  yaw_c,
  roll_c,
  Vpitch_c,
  Vyaw_c,
  Vroll_c,
  pitch_p,
  yaw_p,
  roll_p,
  Fx_f,
  Fy_f,
  Fz_f,
  Fx_l,
  Fy_l,
  Fz_l,
  Ax_f,
  Ay_f,
  Az_f,
  a_g_x,
  a_g_y,
  a_g_z,
  Vpitch_p
};

bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                               const Eigen::MatrixXd &invCovariance,
                               const double nsigmas)
{
  double sqMahalanobis = innovation.dot(invCovariance * innovation);
  double threshold = nsigmas * nsigmas;

  if (sqMahalanobis >= threshold)
  {
    return false;
  }

  return true;
}

void initialize(){
  ROS_INFO("initilaize");
  /*test variable*/
  /*
  svo_pose.pose.pose.position.x = 0.1;
  svo_pose.pose.pose.position.y = 0.2;
  svo_pose.pose.pose.position.z = 0.3;
  imu_data.linear_acceleration.x = 0.1;
  imu_data.linear_acceleration.y = 0.2;
  imu_data.linear_acceleration.z = 9.9;
  */
  /*test variable*/
  double alpha = 1e-3;
  double kappa = 0;
  double beta = 2;
  //const int STATE_SIZE = 19;
  float sigmaCount = (STATE_SIZE << 1) +1; //2L + 1 = 37(19 states)
  sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));
  sigmaPoints_a.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

  //Prepare constants
  //lamda,
  lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
  //ROS_INFO("lamda = %f", lambda_);
  stateWeights_.resize(sigmaCount);
  covarWeights_.resize(sigmaCount);

  // Wi_c, Wi_m
  stateWeights_[0] = lambda_ / (STATE_SIZE + lambda_);
  //stateWeights_[0] = 1 / (sigmaCount);
  covarWeights_[0] =  stateWeights_[0] + (1 - (alpha * alpha) + beta);
  //covarWeights_[0] = 1 / (sigmaCount);
  sigmaPoints_[0].setZero();


  for (size_t i = 1; i < sigmaCount; ++i)
  {
    sigmaPoints_[i].setZero();
    stateWeights_[i] =  1 / (2 * (STATE_SIZE + lambda_));
    //stateWeights_[i] = 1 / (sigmaCount);
    covarWeights_[i] = stateWeights_[i];
  }

  // Initialize Px,P_k-1
  estimateErrorCovariance_(0,0) = 1e-02;// x_c
  estimateErrorCovariance_(1,1) = 1e-02;// y_c
  estimateErrorCovariance_(2,2) = 1e-02;// z_c
  estimateErrorCovariance_(3,3) = 1e-06;// Vx_c
  estimateErrorCovariance_(4,4) = 1e-06;// Vy_c
  estimateErrorCovariance_(5,5) = 1e-06;// Vz_c
  estimateErrorCovariance_(6,6) = 1e-06;// pitch_c
  estimateErrorCovariance_(7,7) = 1e-06;// yaw_c
  estimateErrorCovariance_(8,8) = 1e-06;// roll_c
  estimateErrorCovariance_(9,9) = 1e-06;// Vpitch_c
  estimateErrorCovariance_(10,10) = 1e-06;// Vyaw_c
  estimateErrorCovariance_(11,11) = 1e-06;// Vroll_c
  estimateErrorCovariance_(12,12) = 1e-02;// pitch_p
  estimateErrorCovariance_(13,13) = 1e-02;// yaw_p
  estimateErrorCovariance_(14,14) = 1e-02;// roll_p
  estimateErrorCovariance_(15,15) = 1e-06;// Vpitch_p
  estimateErrorCovariance_(16,16) = 1e-06;// Vyaw_p
  estimateErrorCovariance_(17,17) = 1e-06;// Vroll_p
  estimateErrorCovariance_(18,18) = 1e-02;//F_x_f
  estimateErrorCovariance_(19,19) = 1e-06;//F_y_f
  estimateErrorCovariance_(20,20) = 1e-06;//F_z_f
  estimateErrorCovariance_(21,21) = 1e-06;//F_x_l
  estimateErrorCovariance_(22,22) = 1e-06;//F_y_l
  estimateErrorCovariance_(23,23) = 1e-06;//F_z_l
  estimateErrorCovariance_(24,24) = 1e-06;//a_g_x
  estimateErrorCovariance_(25,25) = 1e-06;//a_g_y
  estimateErrorCovariance_(26,26) = 1e-06;//a_g_z
  estimateErrorCovariance_(27,27) = 1e-06;







  // Initialize state by using first measurement x_0
  state_.setZero();

  uncorrected_ = false;

  measurement.mahalanobisThresh_ = 8;





}

double clamRotation(double rotation)
{
  const double PI = 3.141592653589793;
  const double TAU = 6.283185307179587;
  while (rotation > PI)
  {
    rotation -= TAU;
  }

  while (rotation < -PI)
  {
    rotation += TAU;
  }

  return rotation;

}

void checkData(){
  if(measure_data.pose.position.x != 0 && output_data.force.x != 0 && output_data.Af.x != 0){
    flag = 1;
  }
}

void writeInMeasurement(){

  measurement.measurement_.resize(STATE_SIZE);
  float roll, pitch , yaw;
  float theta_c = 0;
  //const float imu_ax_bias = -0.077781;
  //const float imu_ay_bias = 0.083215;
  Eigen::Matrix3f Rx, Ry, Rz;
  Eigen::Vector3f a_g_inertial;
  Eigen::Vector3f a_g_body;
  Rx.setZero();
  Ry.setZero();
  Rz.setZero();
  a_g_inertial.setZero();

  a_g_inertial(0) = 0;
  a_g_inertial(1) = 0;
  a_g_inertial(2) = a_g;
  //read payload attitude from topic
  roll = payload_roll;
  pitch = payload_pitch;
  yaw = payload_yaw;
  Rx(0,0) = 1;
  Rx(1,0) = 0;
  Rx(2,0) = 0;
  Rx(0,1) = 0;
  Rx(1,1) = cos(roll);
  Rx(1,2) = -sin(roll);
  Rx(0,2) = 0;
  Rx(2,1) = sin(roll);
  Rx(2,2) = cos(roll);

  Ry(0,0) = cos(pitch);
  Ry(1,0) = 0;
  Ry(2,0) = sin(pitch);
  Ry(0,1) = 0;
  Ry(1,1) = 1;
  Ry(1,2) = 0;
  Ry(0,2) = -sin(pitch);
  Ry(2,1) = 0;
  Ry(2,2) = cos(pitch);

  Rz(0,0) = cos(yaw);
  Rz(1,0) = -sin(yaw);
  Rz(2,0) = 0;
  Rz(0,1) = sin(yaw);
  Rz(1,1) = cos(yaw);
  Rz(1,2) = 0;
  Rz(0,2) = 0;
  Rz(2,1) = 0;
  Rz(2,2) = 1;

  a_g_body = Ry*Rx*Rz*a_g_inertial;
  //a_g_body(0) = (sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll)) * 9.8;
  //a_g_body(0) = sin(pitch)*cos(roll)*a_g;

  measurement.measurement_[x_c] = measure_data.pose.position.x;
  measurement.measurement_[y_c] = measure_data.pose.position.y;
  measurement.measurement_[z_c] = measure_data.pose.position.z;
  //ROS_INFO("x_c = %f", measurement.measurement_[x_c]);
  theta_c = atan2(output_data.force.z, output_data.force.x);
  if(output_data.force.x > 0){
  measurement.measurement_[pitch_c] = theta_c + 3.1415926;
  }
  //ROS_INFO("Fx = %f, Fz = %f", output_data.force.x, output_data.force.z);
  //ROS_INFO("theta_c = %f",measurement.measurement_[pitch_c]*180/3.1415926);
  /*
  measurement.measurement_[yaw_c] = ;
  measurement.measurement_[roll_c] = ;
  */
  tf::Quaternion quat1(measure_data.pose.orientation.x, measure_data.pose.orientation.y, measure_data.pose.orientation.z, measure_data.pose.orientation.w);
  double roll_mocap, pitch_mocap, yaw_mocap;
  tf::Matrix3x3(quat1).getRPY(roll_mocap, pitch_mocap, yaw_mocap);

  geometry_msgs::Vector3 rpy_mocap;
  rpy_mocap.x = roll_mocap;
  rpy_mocap.y = pitch_mocap;
  rpy_mocap.z = yaw_mocap;

  measurement.measurement_[pitch_p] = rpy_mocap.y;
  //ROS_INFO("pitch_p = %f", measurement.measurement_[pitch_p]);
  /*
  measurement.measurement_[yaw_p] = ;
  measurement.measurement_[roll_p] = ;
  */


  measurement.measurement_[Fx_f] = output_data.force.x;
  measurement.measurement_[Fy_f] = output_data.force.y;
  measurement.measurement_[Fz_f] = output_data.force.z;

/*
  state_[Fx_f] = output_data.force.x;
  state_[Fy_f] = output_data.force.y;
  state_[Fz_f] = output_data.force.z;
*/


  state_[Ax_f] = output_data.Af.x;
  state_[Ay_f] = output_data.Af.y;
  state_[Az_f] = output_data.Af.z;
  //ROS_INFO("Ax_f = %f, Az_f = %f", state_[Ax_f], state_[Az_f]);

  state_[a_g_x] = a_g_body(0);
  state_[a_g_y] = a_g_body(1);
  state_[a_g_z] = -a_g_body(2);


}

void correct(){
  //ROS_INFO("---correct start---\n");


  //const int STATE_SIZE = 19;
  const double PI = 3.141592653589793;
  const double TAU = 6.283195307179587;

  //correct, calculate sigma points, if uncorrected = true ,than this loop won't be run.
  if (!uncorrected_)
  {
    // caculate square root of (L+lamda)*P_k-1
    weightedCovarSqrt_ = ((STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();
    // First sigma point is the current state
    sigmaPoints_[0] = state_;
    // Generate the sigma points
    // x_i = x + weightedCovarSqrt_ , i = 1, ..., L
    // x_i = x - weightedCovarSqrt_ , i = L+1, ..., 2L
    for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
     {
      sigmaPoints_[sigmaInd + 1] = state_ + weightedCovarSqrt_.col(sigmaInd);
      sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = state_ - weightedCovarSqrt_.col(sigmaInd);
     }
  }


  // We don't want to update everything, so we need to build matrices that only update
  // the measured parts of our state vector

  // First, determine how many state vector values we're updating

  //size_t updateSize = 19 ;

  // Now set up the relevant matrices
  //Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
  Eigen::VectorXd measurementSubset(STATE_SIZE);                        // y
  Eigen::MatrixXd measurementCovarianceSubset(STATE_SIZE, STATE_SIZE);  // Py
  Eigen::MatrixXd stateToMeasurementSubset(STATE_SIZE, STATE_SIZE);     // H
  Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, STATE_SIZE);             // K
  Eigen::VectorXd innovationSubset(STATE_SIZE);                         // y - Hx
  Eigen::VectorXd predictedMeasurement(STATE_SIZE);
  Eigen::VectorXd sigmaDiff(STATE_SIZE);
  Eigen::MatrixXd predictedMeasCovar(STATE_SIZE, STATE_SIZE);
  Eigen::MatrixXd crossCovar(STATE_SIZE, STATE_SIZE);

  std::vector<Eigen::VectorXd> sigmaPointMeasurements(sigmaPoints_.size(), Eigen::VectorXd(STATE_SIZE));


  //stateSubset.setZero();
  measurementSubset.setZero();
  measurementCovarianceSubset.setZero();
  stateToMeasurementSubset.setZero();
  kalmanGainSubset.setZero();
  innovationSubset.setZero();
  predictedMeasurement.setZero();
  predictedMeasCovar.setZero();
  crossCovar.setZero();

  // Now build the sub-matrices from the full-sized matrices

  for (size_t i = 0; i < STATE_SIZE; ++i){
    measurementSubset(i) = measurement.measurement_[i];
    //stateSubset(i) = state_(i);
    //measurementCovarianceSubset(i,i) = measurement.covariance_(i,i);

   }

  // The state-to-measurement function, H, will now be a measurement_size x full_state_size
  // matrix, with ones in the (i, i) locations of the values to be updated
  stateToMeasurementSubset(0,0) = 1;
  stateToMeasurementSubset(1,1) = 1;
  stateToMeasurementSubset(2,2) = 1;
  stateToMeasurementSubset(3,3) = 0;
  stateToMeasurementSubset(4,4) = 0;
  stateToMeasurementSubset(5,5) = 0;
  stateToMeasurementSubset(6,6) = 1;
  stateToMeasurementSubset(7,7) = 1;
  stateToMeasurementSubset(8,8) = 1;
  stateToMeasurementSubset(9,9) = 0;
  stateToMeasurementSubset(10,10) = 0;
  stateToMeasurementSubset(11,11) = 0;
  stateToMeasurementSubset(12,12) = 1;
  stateToMeasurementSubset(13,13) = 1;
  stateToMeasurementSubset(14,14) = 1;
  stateToMeasurementSubset(15,15) = 1;
  stateToMeasurementSubset(16,16) = 1;
  stateToMeasurementSubset(17,17) = 1;
  stateToMeasurementSubset(18,18) = 0;
  stateToMeasurementSubset(19,19) = 0;
  stateToMeasurementSubset(20,20) = 0;
  stateToMeasurementSubset(21,21) = 0;
  stateToMeasurementSubset(22,22) = 0;
  stateToMeasurementSubset(23,23) = 0;
  stateToMeasurementSubset(24,24) = 0;
  stateToMeasurementSubset(25,25) = 0;
  stateToMeasurementSubset(26,26) = 0;
  stateToMeasurementSubset(27,27) = 0;



  //The measurecovariance subset R

  measurementCovarianceSubset(0,0) = 0.2;
  measurementCovarianceSubset(1,1) = 0.2;
  measurementCovarianceSubset(2,2) = 0.2;
  measurementCovarianceSubset(6,6) = 0.2;
  measurementCovarianceSubset(7,7) = 0.2;
  measurementCovarianceSubset(8,8) = 0.2;
  measurementCovarianceSubset(12,12) = 0.2;
  measurementCovarianceSubset(13,13) = 0.2;
  measurementCovarianceSubset(14,14) = 0.2;
  measurementCovarianceSubset(15,15) = 0.2;
  measurementCovarianceSubset(16,16) = 0.2;
  measurementCovarianceSubset(17,17) = 0.2;
  measurementCovarianceSubset(3,3) = measurementCovarianceSubset(4,4) = measurementCovarianceSubset(5,5) = measurementCovarianceSubset(9,9) = measurementCovarianceSubset(10,10) = measurementCovarianceSubset(11,11) = measurementCovarianceSubset(18,18) = measurementCovarianceSubset(19,19) = measurementCovarianceSubset(20,20) = measurementCovarianceSubset(21,21) = measurementCovarianceSubset(22,22) =  measurementCovarianceSubset(23,23) = measurementCovarianceSubset(24,24) = measurementCovarianceSubset(25,25) = measurementCovarianceSubset(26,26) = measurementCovarianceSubset(27,27) = 0.4;

  // (5) Generate sigma points, use them to generate a predicted measurement,y_k_hat-
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * sigmaPoints_[sigmaInd];
    // y = sum of (wi*yi)
    predictedMeasurement.noalias() += stateWeights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];
  }

  // (6) Use the sigma point measurements and predicted measurement to compute a predicted
  // measurement covariance matrix P_yy and a state/measurement cross-covariance matrix P_xy.

  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;//Y(i)_k|k-1 - y_k_hat-
    predictedMeasCovar.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());//P_y_k~_y_k_~
    crossCovar.noalias() += covarWeights_[sigmaInd] * ((sigmaPoints_[sigmaInd] - state_) * sigmaDiff.transpose());//P_x_k_y_k
  }

  //check p_yy
  for (int i = 3; i < 12 ; i++){
    for (int j = 0; j < 3 ; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 0; i < 28; i++){
    for (int j = 3; j < 12; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 3; i < 12; i++){
    for (int j = 12; j < 18; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 0;i < 28; i++){
    for (int j = 18; j < 28; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  //check p_xy
  for (int i = 0; i < 28; i++){
    for (int j =3; j < 12; j++){
      crossCovar(i,j) = 0;
    }
  }
  for (int i = 0; i < 28; i++){
    for (int j = 18; j <28; j++){
      crossCovar(i,j) = 0;
    }
  }

  // (7) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_x_k_y_k * (P_y_k~_y_k_~ + R)^-1
  // kalman gain :https://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively
  Eigen::MatrixXd invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();

  kalmanGainSubset = crossCovar * invInnovCov;


  // (8) Apply the gain to the difference between the actual and predicted measurements: x = x + K(y - y_hat)
  // y - y_hat


  innovationSubset = (measurementSubset - predictedMeasurement);


  // Wrap angles in the innovation
  while (innovationSubset(roll_p) < -PI)
   {
   innovationSubset(roll_p) += TAU;
   }

   while (innovationSubset(roll_p) > PI)
   {
    innovationSubset(roll_p) -= TAU;
   }

   while (innovationSubset(yaw_p) < -PI)
    {
    innovationSubset(yaw_p) += TAU;
    }

    while (innovationSubset(yaw_p) > PI)
    {
     innovationSubset(yaw_p) -= TAU;
    }

    while (innovationSubset(pitch_p) < -PI)
     {
     innovationSubset(pitch_p) += TAU;
     }

     while (innovationSubset(pitch_p) > PI)
     {
      innovationSubset(pitch_p) -= TAU;
     }
     double sqMahalanobis = innovationSubset.dot(invInnovCov * innovationSubset);
     double threshold = 1 * 1;



  // (8.1) Check Mahalanobis distance of innovation
  //if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThresh_))
  //{
    // x = x + K*(y - y_hat)
    state_.noalias() += kalmanGainSubset * innovationSubset;
    //ROS_INFO("Vc: x = %f, y = %f, z = %f", state_[Vx_c], state_[Vy_c], state_[Vz_c]);
    //ROS_INFO("Vc : x = %f, y = %f , z = %f", state_[Vx_c], state_[Vy_c], state_[Vz_c]);
    //ROS_INFO("w_p = %f", state_[Vpitch_p]);
    //ROS_INFO("F_l: x = %f, z = %f", state_[Fx_l], state_[Fz_l]);
    output_vc.pose.position.x = state_[Vx_c];
    output_vc.pose.position.y = state_[Vy_c];
    output_vc.pose.position.z = state_[Vz_c];

    output_omegac.pose.position.y = state_[Vpitch_c];

    output_leader.pose.position.x = state_[Fx_l];
    output_leader.pose.position.y = state_[Fy_l];
    output_leader.pose.position.z = state_[Fz_l];




    // (9) Compute the new estimate error covariance P = P - (K * P_yy * K')
    estimateErrorCovariance_.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

    //ROS_INFO("predict::estimate = %f",estimateErrorCovariance_(0,0));
    //ROS_INFO("kalman gain = %f", kalmanGainSubset(0,0));
    //ROS_INFO("predictedMeas = %f", predictedMeasCovar(0,0));

    //wrapStateAngles();
    state_(roll_p) = clamRotation(state_(roll_p));
    state_(yaw_p) = clamRotation(state_(yaw_p));
    state_(pitch_p) = clamRotation(state_(pitch_p));

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;
    measurement.mahalanobisThresh_ = 2;
    //ROS_INFO("---correct success!!---\n");

  //}
  //else{
    measurement.mahalanobisThresh_ = 8;
    //ROS_INFO("---correct fail!!---\n");
  //}


}

void predict(const double referenceTime, const double delta)
{
  //ROS_INFO("---Predict start---");
  Eigen::MatrixXd transferFunction_(STATE_SIZE,STATE_SIZE);
  Eigen::MatrixXd process_noise_m(STATE_SIZE,STATE_SIZE);
  double m = 1.2,m_p = 0.6;
  //const int STATE_SIZE = 19;

  double roll = state_(roll_p);
  double pitch = state_(pitch_p);
  double yaw = state_(yaw_p);

  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);

  double vpitch_c = state_[Vpitch_c];
  double vpitch_p = state_[Vpitch_p];
  //l
  double l_x = measure_data.pose.position.x - mocap_pose.pose.position.x ,l_z = measure_data.pose.position.z - mocap_pose.pose.position.z;//cable length

  double r_cp;

  double theta_p = state_[pitch_p];
  double theta_c = measurement.measurement_[pitch_c];

  double theta_f_pitch = output_data.theta.y;

  double I_p = 0.0;


  double l = 0.3; // payload length
  double g = 9.8;

  double theta_d = atan2(state_[Fz_l], state_[Fx_l]);
  //std::cout << theta_c << std::endl;
  //leader force angle
  //transfer function

  I_p = m_p*(0.04*0.04+0.3*0.3)/12;

  //p_c_dot = v_c
  transferFunction_(x_c,x_c) = transferFunction_(y_c,y_c) = transferFunction_(z_c,z_c) = 1;
  transferFunction_(x_c,Vx_c) = cy * cp * delta;
  transferFunction_(x_c,Vy_c) = (cy * sp * sr - sy * cr) * delta;
  transferFunction_(x_c,Vz_c) = (cy * sp * cr + sy * sr) * delta;
  transferFunction_(y_c,Vx_c) = sy * cp * delta;
  transferFunction_(y_c,Vy_c) = (sy * sp * sr + cy * cr) * delta;
  transferFunction_(y_c,Vz_c) = (sy * sp * cr - cy * sr) * delta;
  transferFunction_(z_c,Vx_c) = -sp * delta;
  transferFunction_(z_c,Vy_c) = cp * sr * delta;
  transferFunction_(z_c,Vz_c) = cp * cr * delta;
/*
  //v_c_dot = a_c = a_f + ...
  transferFunction_(Vx_c,Vx_c) = transferFunction_(Vy_c,Vy_c) = transferFunction_(Vz_c,Vz_c) = 1;
  transferFunction_(Vx_c,Ax_f) = cos(theta_f)*delta;
  transferFunction_(Vx_c,Az_f) = sin(theta_f)*delta;
  transferFunction_(Vx_c,Vpitch_c) = vpitch_c*l_x*delta;
  transferFunction_(Vz_c,Ax_f) = -sin(theta_f)*delta;
  transferFunction_(Vz_c,Az_f) = cos(theta_f)*delta;
  transferFunction_(Vz_c,Vpitch_c) = 1*vpitch_c*l_z*delta;
  */
  //theta_c_dot = omega_c
  transferFunction_(pitch_c,pitch_c) = 1;
  transferFunction_(pitch_c,Vpitch_c) = delta;
  //omega_dot = 0
  //transferFunction_(Vpitch_c,Vpitch_c) = 1;
  //theta_p_dot = omega_p
  transferFunction_(pitch_p,pitch_p) = 1;
  transferFunction_(pitch_p,Vpitch_p) = 1*delta;
  //omeag_p_dot = alpha_p
  /*
  transferFunction_(Vpitch_p,Vpitch_p) = 1 + vpitch_p*delta;
  transferFunction_(Vpitch_p,Fx_f) = 1/(r_cp*m_p)*delta;
  transferFunction_(Vpitch_p,Fx_l) = 1/(r_cp*m_p)*delta;
  transferFunction_(Vpitch_p,Ax_f) = -1*cos(theta_f_pitch)/r_cp*delta;
  transferFunction_(Vpitch_p,Az_f) = -1*sin(theta_f_pitch)/r_cp*delta;
  transferFunction_(Vpitch_p,Vpitch_c) = -1*vpitch_p*l_x;
*/
  //F_l
  transferFunction_(Fx_l,Fx_l) = 1;
  transferFunction_(Fy_l,Fy_l) = 1;
  transferFunction_(Fz_l,Fz_l) = 1;

  transferFunction_(Fx_f,Fx_f) = 1;
  transferFunction_(Fy_f,Fy_f) = 1;
  transferFunction_(Fz_f,Fz_f) = 1;

  process_noise_m(0,0) = 0.05;//x_c
  process_noise_m(1,1) = 0.05;//y_c
  process_noise_m(2,2) = 0.06;//z_c
  process_noise_m(3,3) = 0.3;//Vx_c
  process_noise_m(4,4) = 0.3;//Vy_c
  process_noise_m(5,5) = 0.6;//Vz_c
  process_noise_m(6,6) = 0.05;//pitch_c
  process_noise_m(7,7) = 0.05;//yaw_c
  process_noise_m(8,8) = 0.04;//roll_c
  process_noise_m(9,9) = 0.5;//Vpitch_c
  process_noise_m(10,10) = 0.4;//Vyaw_c
  process_noise_m(11,11) = 0.4;//Vroll_c
  process_noise_m(12,12) = 0.05;//pitch_p
  process_noise_m(13,13) = 0.05;//yaw_p
  process_noise_m(14,14) = 0.08;//roll_p
  process_noise_m(15,15) = 0.05;//Fx_f
  process_noise_m(16,16) = 0.06;//Fy_f
  process_noise_m(17,17) = 0.04;//Fz_f
  process_noise_m(18,18) = 0.5;//Fx_l
  process_noise_m(19,19) = 0.5;//Fy_l
  process_noise_m(20,20) = 0.4;//Fz_l
  process_noise_m(21,21) = 0.01;//Ax_f
  process_noise_m(22,22) = 0.01;//Ay_f
  process_noise_m(23,23) = 0.01;//Az_f
  process_noise_m(24,24) = 0.01;//a_g_x
  process_noise_m(25,25) = 0.01;//a_g_y
  process_noise_m(26,26) = 0.01;//a_g_z
  process_noise_m(27,27) = 1.5;//V_pitch_p
  // (1) Take the square root of a small fraction of the estimateErrorCovariance_ using LL' decomposition
  // caculate square root of (L+lamda)*P_k-1
  // This will be a diagonal matrix (19*19)

  weightedCovarSqrt_ = ((STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();

  // (2) Compute sigma points *and* pass them through the transfer function to save
  // the extra loop

  // First sigma point(through transferfunction) is the current state
  // x_k|k-1(0)
  // sigmaPoint_[0][0~14]

  sigmaPoints_[0] = transferFunction_ * state_;




  // Next STATE_SIZE sigma points are state + weightedCovarSqrt_[ith column]
  // STATE_SIZE sigma points after that are state - weightedCovarSqrt_[ith column]
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {
    sigmaPoints_[sigmaInd + 1] = transferFunction_ * (state_ + weightedCovarSqrt_.col(sigmaInd));
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = transferFunction_ * (state_ - weightedCovarSqrt_.col(sigmaInd));
  }
  //generate sigma points
  sigmaPoints_a[0] = state_;
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {
    sigmaPoints_a[sigmaInd + 1] = state_ + weightedCovarSqrt_.col(sigmaInd);
    sigmaPoints_a[sigmaInd + 1 + STATE_SIZE] = state_ - weightedCovarSqrt_.col(sigmaInd);
  }
  /*
  printf("---sigmaPoints---\n");
    for (int i = 0; i < 57 ; i++){
      for (int j = 0; j< 28 ; j++){
        printf("%f ", sigmaPoints_a[i][j]);
      }
      printf("\n");
    }
  printf("\n");
  */
  //v_c_dot = ...
  sigmaPoints_a[0][Vx_c] = sigmaPoints_a[0][Vx_c] + ((1/m_p)*(sigmaPoints_a[0][Fx_f] + sigmaPoints_a[0][Fx_l]) - sigmaPoints_a[0][Vpitch_p]*sigmaPoints_a[0][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[0][Fx_l]*sigmaPoints_a[0][Fx_l]+sigmaPoints_a[0][Fz_l]*sigmaPoints_a[0][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[0][Fx_f]*sigmaPoints_a[0][Fx_f]+sigmaPoints_a[0][Fz_f]*sigmaPoints_a[0][Fz_f])))*delta;
  sigmaPoints_a[0][Vz_c] = sigmaPoints_a[0][Vz_c] + ((1/m_p)*(sigmaPoints_a[0][Fz_f] + sigmaPoints_a[0][Fz_l] - m_p * g) - sigmaPoints_a[0][Vpitch_p]*sigmaPoints_a[0][Vpitch_p] * r_cp * sin(theta_p) + (1/I_p) * r_cp * sin(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[0][Fx_l]*sigmaPoints_a[0][Fx_l]+sigmaPoints_a[0][Fz_l]*sigmaPoints_a[0][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[0][Fx_f]*sigmaPoints_a[0][Fx_f]+sigmaPoints_a[0][Fz_f]*sigmaPoints_a[0][Fz_f])))*delta;
  //std::cout << sigmaPoints_a[0][Vx_c] << std::endl;
  sigmaPoints_[0][Vx_c] = sigmaPoints_a[0][Vx_c];
  sigmaPoints_[0][Vz_c] = sigmaPoints_a[0][Vz_c];


  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {    
    sigmaPoints_a[sigmaInd + 1][Vx_c] = sigmaPoints_a[sigmaInd + 1][Vx_c] + ((1/m_p)*(sigmaPoints_a[sigmaInd + 1][Fx_f] + sigmaPoints_a[sigmaInd + 1][Fx_l]) - sigmaPoints_a[sigmaInd + 1][Vpitch_p]*sigmaPoints_a[sigmaInd + 1][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_l]*sigmaPoints_a[sigmaInd + 1][Fx_l]+sigmaPoints_a[sigmaInd + 1][Fz_l]*sigmaPoints_a[sigmaInd + 1][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_f]*sigmaPoints_a[sigmaInd + 1][Fx_f]+sigmaPoints_a[sigmaInd + 1][Fz_f]*sigmaPoints_a[sigmaInd + 1][Fz_f])))*delta;
    sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vx_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vx_c] + ((1/m_p)*(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f] + sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]) - sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f])))*delta;
    sigmaPoints_a[sigmaInd + 1][Vz_c] = sigmaPoints_a[sigmaInd + 1][Vz_c] + ((1/m_p)*(sigmaPoints_a[sigmaInd + 1][Fz_f] + sigmaPoints_a[sigmaInd + 1][Fz_l] - m_p * g) - sigmaPoints_a[sigmaInd + 1][Vpitch_p]*sigmaPoints_a[sigmaInd + 1][Vpitch_p] * r_cp * sin(theta_p) + (1/I_p) * r_cp * sin(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_l]*sigmaPoints_a[sigmaInd + 1][Fx_l]+sigmaPoints_a[sigmaInd + 1][Fz_l]*sigmaPoints_a[sigmaInd + 1][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_f]*sigmaPoints_a[sigmaInd + 1][Fx_f]+sigmaPoints_a[sigmaInd + 1][Fz_f]*sigmaPoints_a[sigmaInd + 1][Fz_f])))*delta;
    sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vz_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vz_c] + ((1/m_p)*(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f] + sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l] - m_p * g) - sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p] * r_cp * cos(theta_p) + (1/I_p) * r_cp * sin(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f])))*delta;

    sigmaPoints_[sigmaInd + 1][Vx_c] = sigmaPoints_a[sigmaInd + 1][Vx_c];
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE][Vx_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vx_c];
    sigmaPoints_[sigmaInd + 1][Vz_c] = sigmaPoints_a[sigmaInd + 1][Vz_c];
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE][Vz_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vz_c];
  }
  /*
  for (int i = 0; i < 2*STATE_SIZE +1 ; i++){
    printf("%f ", sigmaPoints_[i][Vx_c]);
  }
  */

  // omega_c_dot = ...
  Eigen::Vector3d A_f_B , A_f_I;
  A_f_B.setZero();
  A_f_I.setZero();
  A_f_B << sigmaPoints_a[0][Ax_f], sigmaPoints_a[0][Ay_f], sigmaPoints_a[0][Az_f];
  A_f_I = R_B_I * A_f_B;
  sigmaPoints_a[0][Vpitch_c] = sigmaPoints_a[0][Vpitch_c] + ((1/l_x) * ((1/m_p)*(sigmaPoints_a[0][Fx_f] + sigmaPoints_a[0][Fx_l]) - sigmaPoints_a[0][Vpitch_p]*sigmaPoints_a[0][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[0][Fx_l]*sigmaPoints_a[0][Fx_l]+sigmaPoints_a[0][Fz_l]*sigmaPoints_a[0][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[0][Fx_f]*sigmaPoints_a[0][Fx_f]+sigmaPoints_a[0][Fz_f]*sigmaPoints_a[0][Fz_f])) - A_f_I(0) - sigmaPoints_a[0][Vpitch_p]*sigmaPoints_a[0][Vpitch_p]*l_x))*delta;
  sigmaPoints_[0][Vpitch_c] = sigmaPoints_a[0][Vpitch_c];
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {

    A_f_B.setZero();
    A_f_I.setZero();
    A_f_B << sigmaPoints_a[sigmaInd + 1][Ax_f], sigmaPoints_a[sigmaInd + 1][Ay_f], sigmaPoints_a[sigmaInd + 1][Az_f];
    A_f_I = R_B_I * A_f_B;
    sigmaPoints_a[sigmaInd + 1][Vpitch_c] = sigmaPoints_a[sigmaInd + 1][Vpitch_c] + ((1/l_x) * ((1/m_p)*(sigmaPoints_a[sigmaInd + 1][Fx_f] + sigmaPoints_a[sigmaInd + 1][Fx_l]) - sigmaPoints_a[sigmaInd + 1][Vpitch_p]*sigmaPoints_a[sigmaInd + 1][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_l]*sigmaPoints_a[sigmaInd + 1][Fx_l]+sigmaPoints_a[sigmaInd + 1][Fz_l]*sigmaPoints_a[sigmaInd + 1][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1][Fx_f]*sigmaPoints_a[sigmaInd + 1][Fx_f]+sigmaPoints_a[sigmaInd + 1][Fz_f]*sigmaPoints_a[sigmaInd + 1][Fz_f])) - A_f_I(0) - sigmaPoints_a[sigmaInd + 1][Vpitch_p]*sigmaPoints_a[sigmaInd + 1][Vpitch_p]*l_x))*delta;
    sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_c] + ((1/l_x) * ((1/m_p)*(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f] + sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]) - sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p] * r_cp * cos(theta_p) - (1/I_p) * r_cp * cos(theta_p) * (-(0.5*l)*sin(theta_p+theta_d)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]) + (0.5*l)*sin(theta_p+theta_c)*sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]+sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f])) - A_f_I(0) - sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p]*l_x))*delta;

    sigmaPoints_[sigmaInd + 1][Vpitch_c] = sigmaPoints_a[sigmaInd + 1][Vpitch_c];
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE][Vpitch_c] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_c];
  }
  /*
  for (int i = 0; i < 2*STATE_SIZE +1 ; i++){
    printf("%f ", sigmaPoints_[i][Vpitch_c]);
  }
  printf("\n");
  */

  //omega_p_dot = ...
    double Fl1,Fl2,Ff1,Ff2;
    Fl1 = Fl2 = Ff1 = Ff2 = 0;
    Ff1 = sqrt(sigmaPoints_a[0][Fx_f]*sigmaPoints_a[0][Fx_f] + sigmaPoints_a[0][Fz_f] * sigmaPoints_a[0][Fz_f]);
    Fl1 = sqrt(sigmaPoints_a[0][Fx_l]*sigmaPoints_a[0][Fx_l] + sigmaPoints_a[0][Fz_l] * sigmaPoints_a[0][Fz_l]);
    sigmaPoints_a[0][Vpitch_p] = sigmaPoints_a[0][Vpitch_p] + (1/I_p) * (-(0.5*l)*sin(theta_p+theta_d) * Fl1 + (0.5*l)*sin(theta_p+theta_c) * Ff1) * delta;
    sigmaPoints_[0][Vpitch_p] = sigmaPoints_a[0][Vpitch_p];
    /*
    std::cout << "---Vpitch_p---" << std::endl;
    std::cout << sigmaPoints_[0][Vpitch_p] << std::endl;
*/
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd){
    Fl1 = Fl2 = Ff1 = Ff2 = 0;
    Ff1 = sqrt(sigmaPoints_a[sigmaInd + 1][Fx_f]*sigmaPoints_a[sigmaInd + 1][Fx_f] + sigmaPoints_a[sigmaInd + 1][Fz_f] * sigmaPoints_a[sigmaInd + 1][Fz_f]);
    Ff2 = sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_f] + sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f] * sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_f]);
    Fl1 = sqrt(sigmaPoints_a[sigmaInd + 1][Fx_l]*sigmaPoints_a[sigmaInd + 1][Fx_l] + sigmaPoints_a[sigmaInd + 1][Fz_l] * sigmaPoints_a[sigmaInd + 1][Fz_l]);
    Fl2 = sqrt(sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l]*sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fx_l] + sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l] * sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Fz_l]);
    sigmaPoints_a[sigmaInd + 1][Vpitch_p] = sigmaPoints_a[sigmaInd + 1][Vpitch_p] + (1/I_p) * (-(0.5*l)*sin(theta_p+theta_d) * Fl1 + (0.5*l)*sin(theta_p+theta_c) * Ff1) * delta;
    sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p] + (1/I_p) * (-(0.5*l)*sin(theta_p+theta_d) * Fl2 + (0.5*l)*sin(theta_p+theta_c) * Ff2) * delta;

    sigmaPoints_[sigmaInd + 1][Vpitch_p] = sigmaPoints_a[sigmaInd + 1][Vpitch_p];
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE][Vpitch_p] = sigmaPoints_a[sigmaInd + 1 + STATE_SIZE][Vpitch_p];
  }

    /*
  for (int i = 0; i < 2*STATE_SIZE +1 ; i++){
    printf("%f ", sigmaPoints_[i][Vpitch_p]);
  }
  printf("\n");
*/
  /*
  std::cout << "---sigmaPoints_a---" << std::endl;
  for(int i=0 ; i< sigmaPoints_a.size();i++){
     std::cout << sigmaPoints_a[i] << std::endl;
  }
*/

  // (3) Sum the weighted sigma points to generate a new state prediction
  // x_k_hat- = w_im * x_k|k-1
  state_.setZero();
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    state_.noalias() += stateWeights_[sigmaInd] * sigmaPoints_[sigmaInd];
  }


  // (4) Now us the sigma points and the predicted state to compute a predicted covariance P_k-
  estimateErrorCovariance_.setZero();


  Eigen::VectorXd sigmaDiff(STATE_SIZE);
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = (sigmaPoints_[sigmaInd] - state_);
    //ROS_INFO("sigmapoint = %f", sigmaPoints_[0][0]);
    //ROS_INFO("sigmaDiff = %f", sigmaDiff[0]);
    estimateErrorCovariance_.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
  }
  estimateErrorCovariance_ = estimateErrorCovariance_ + process_noise_m;


// Mark that we can keep these sigma points
      uncorrected_ = true;
      //ROS_INFO("predict ok!");


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "payload");
  ros::NodeHandle nh;
  //get param
  string topic_measure, topic_mocap;

  ros::param::get("~topic_measure", topic_measure);
  ros::param::get("~topic_mocap", topic_mocap);
  //ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, svo_cb);
  ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_mocap, 1, mocap_cb);
  //ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_imu, 2, imu_cb);
  //ros::Subscriber vfr_sub = nh.subscribe<mavros_msgs::VFR_HUD>(topic_thrust, 2, vfr_cb);
  //ros::Publisher output_pub = nh.advertise<UKF::output>("/output", 10);
  ros::Subscriber measurement_sub = nh.subscribe<geometry_msgs::PoseStamped>( topic_measure, 1, measure_cb);
  ros::Subscriber output_sub = nh.subscribe<UKF::output>("/output", 1, output_cb);
  //ros::Publisher output_vc_pub = nh.advertise<geometry_msgs::PoseStamped>("/output_vc", 10);
  //ros::Publisher output_omega_pub = nh.advertise<geometry_msgs::PoseStamped>("/output_omegac", 10);
  //ros::Publisher output_leader_pub = nh.advertise<geometry_msgs::PoseStamped>("/output_leader", 10);
  initialize();
  int count = 0;
  ros::Rate rate(50);
  while(ros::ok()){
    //output_data.header.stamp = ros::Time::now();

    //imu_data.header.stamp = ros::Time::now();
    //svo_pose.header.stamp = ros::Time::now();
    //measure_data.pose.position.x = 1;
    checkData();
    if(flag == 1)
    {
    writeInMeasurement();
    predict(1,0.02);
    correct();
    //output_pub.publish(output);
    //output_vc_pub.publish(output_vc);
    //output_omega_pub.publish(output_omegac);
    //output_leader_pub.publish(output_leader);
    }



    ros::spinOnce();
    rate.sleep();


  }
}

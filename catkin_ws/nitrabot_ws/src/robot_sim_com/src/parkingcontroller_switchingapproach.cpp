#include "parkingcontroller_switchingapproach.h"
#include "robotdata.h"
#include <math.h>
#include "ros/ros.h"

using namespace controller;
using namespace industrial_robot;

ParkingController::ParkingController(double y_init, double x_ref, double y_ref, double phi_ref, \
                                     double k_x, double k_y, double k_phix, double k_phiy, double k_p, \
                                     double eps_y, double eps_phi)
{
  this->y_init = y_init;
  this->x_ref = x_ref;
  this->y_ref = y_ref;
  this->phi_ref = phi_ref * RobotParams::deg_to_rad;
  this->k_x = k_x;
  this->k_y = k_y;
  this->k_phix = k_phix;
  this->k_phiy = k_phiy;
  this->k_p = k_p;
  this->eps_y = eps_y;
  this->eps_phi = eps_phi * RobotParams::deg_to_rad;
  this->q = 1;
}

ParkingController::~ParkingController(){}

double * ParkingController::Linearization(double w1, double w2, double yaw_rad)
{
  double theta_r_dot;
  double theta_l_dot;
  static double ret[2];

  if (q == 1)
  {
    theta_r_dot = (RobotParams::wheel_to_center * w1) / RobotParams::wheel_radius;
    theta_l_dot = -(RobotParams::wheel_to_center * w1) / RobotParams::wheel_radius;
  }
  else if (q == 3)
  {
    theta_r_dot = w1 / (cos(yaw_rad) * RobotParams::wheel_radius) + (RobotParams::wheel_to_center * w2) / RobotParams::wheel_radius;
    theta_l_dot = w1 / (cos(yaw_rad) * RobotParams::wheel_radius) - (RobotParams::wheel_to_center * w2) / RobotParams::wheel_radius;
  }
  else if (q == 2)
  {
    theta_r_dot = w1 / (sin(yaw_rad) * RobotParams::wheel_radius) + (RobotParams::wheel_to_center * w2) / RobotParams::wheel_radius;
    theta_l_dot = w1 / (sin(yaw_rad) * RobotParams::wheel_radius) - (RobotParams::wheel_to_center * w2) / RobotParams::wheel_radius;
  }
  ret[0] = theta_r_dot;
  ret[1] = theta_l_dot;
  return ret;
}

double * ParkingController::ControlOutput(double err_x, double err_y, double err_phi)
{
  double w1, w2;
  static double ret[2];
  if (q == 1)
  {
    w1 = k_p * err_phi;
    w2 = w1;
  }
  else if (q == 3)
  {
    w1 = k_x * err_x;
    w2 = k_phix * err_phi;
  }
  else if (q == 2)
  {
    w1 = k_y * err_y;
    w2 = k_phiy * err_phi;
  }
  ret[0] = w1;
  ret[1] = w2;
  return ret;
}

void ParkingController::SwitchSystem(double err_x, double err_y, double err_phi)
{
  if (q == 1)
  {
    if (fabs(err_phi) <= eps_phi && fabs(err_y) > eps_y){q = 2;}
    else if (fabs(err_phi) <= eps_phi && fabs(err_y) <= eps_y){q = 3;}
    else {q = 1;}
  }
  else if (q == 2)
  {
    if (fabs(err_phi) <= eps_phi && fabs(err_y) <= eps_y){q = 3;}
    else {q = 2;}
  }
}

void ParkingController::ChangeReference()
{
  if (q == 1)
  {
    if (y_init < -eps_y){phi_ref = 90.0;}
    else if (y_init > eps_y){phi_ref = 270.0;}
    else {phi_ref = 0.0;}
  }
  else {phi_ref = 0.0;}
  phi_ref *= RobotParams::deg_to_rad;
}

double * ParkingController::CalcError(double x_pos, double y_pos, double yaw_rad)
{
  static double ret[3];
  double err_x = x_ref - x_pos;
  ret[0] = err_x;
  double err_y = y_ref - y_pos;
  ret[1] = err_y;

  /*
  // check if phi ref = 0 (if thats the case we have to consider the noise of the imu
  // it could happen that the robot jumps between 2pi and 0 if its near the reference which would produce a big error (improved with function ConvertError)
  if (phi_ref == 0.0 && yaw_rad > 180 * RobotParams::deg_to_rad)
  {
    yaw_rad -= 360 * RobotParams::deg_to_rad;
  }
  double err_phi = phi_ref - yaw_rad;
  */
  double err_phi = phi_ref - ConvertYaw(yaw_rad);
  ret[2] = err_phi;
  //static double ret [3] = {err_x, err_y, err_phi};
  ROS_INFO("x error:\t[%f]",ret[0]);
  ROS_INFO("y error:\t[%f]",ret[1]);
  ROS_INFO("phi error:\t[%f]",ret[2]);
  return ret;
}

//circular error for phi
double ParkingController::ConvertYaw(double yaw_rad)
{
  if ((yaw_rad - phi_ref) > 180 * RobotParams::deg_to_rad)
  {
    yaw_rad -= 360 * RobotParams::deg_to_rad;
  }
  else if ((yaw_rad - phi_ref) < -180 * RobotParams::deg_to_rad)
  {
    yaw_rad += 360 * RobotParams::deg_to_rad;
  }
  return yaw_rad;
}

double * ParkingController::VelocityCommands(double x_pos, double y_pos, double yaw_rad)
{
  static double ret[2];
  ChangeReference();
  double *error = CalcError(x_pos, y_pos, yaw_rad);
  ROS_INFO("q:\t[%d]",q);
  SwitchSystem(error[0], error[1], error[2]);
  double *pseudo_input = ControlOutput(error[0], error[1], error[2]);
  double *theta_dot = Linearization(pseudo_input[0], pseudo_input[1], yaw_rad);
  ret[0] = theta_dot[0];
  ret[1] = theta_dot[1];
  return ret;
}


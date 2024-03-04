#ifndef PARKINGCONTROLLER_SWITCHINGAPPROACH_H
#define PARKINGCONTROLLER_SWITCHINGAPPROACH_H
// The coordinate system is in ROS different than the one which is used in C#
// different switches to avoid coordinate transformations
namespace controller
{
  class ParkingController
  {
  private:
    int q;

    double y_init;
    // should be changeable so it can be updated after new detection of the parking spot
    double x_ref, y_ref, phi_ref;
    // should be written as ROS params in the future to dynamically change them
    double k_x, k_y, k_phix, k_phiy, k_p;
    double eps_y, eps_phi;

    // linearizes the current system for the controller
    double * Linearization(double w1, double w2, double yaw_rad);
    // calculates the control output for the robot
    double * ControlOutput(double err_x, double err_y, double err_phi);
    // switches the sytem if the epsilon is satisfied
    void SwitchSystem(double err_x, double err_y, double err_phi);
    // changes the phi references
    void ChangeReference();
    // calculates the error for the robot (e = ref - pos)
    double * CalcError(double x_pos, double y_pos, double yaw_rad);

    // converts the yawrad for the curcular error
    double ConvertYaw(double yaw_rad);

  public:
    ParkingController(double y_init, double x_ref, double y_ref, double phi_ref, \
                      double k_x, double k_y, double k_phix, double k_phiy, double k_p, \
                      double eps_y, double eps_phi);

    ~ParkingController();

    // main function which returns the velocities in m/s for the robot
    double * VelocityCommands(double x_pos, double y_pos, double yaw_rad);
  };
}


#endif // PARKINGCONTROLLER_SWITCHINGAPPROACH_H

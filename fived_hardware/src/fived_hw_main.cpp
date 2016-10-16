#include <fived_hardware/fived_hw_control_loop.h>
#include <fived_hardware/fived_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fived_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<FiveDHW> fived_hw_interface
    (new FiveDHW(nh));
  fived_hw_interface->init();

  // Start the control loop
  FiveD_HW_Loop control_loop(nh, fived_hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}

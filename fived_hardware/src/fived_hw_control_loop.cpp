#include <fived_hardware/fived_hw_control_loop.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

FiveD_HW_Loop::FiveD_HW_Loop(
    ros::NodeHandle& nh, boost::shared_ptr<FiveDHW> hardware_interface)
  : nh_(nh), hardware_interface_(hardware_interface)
{
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Load rosparams
  ros::NodeHandle rpsnh(nh, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loop_hz_);
  error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold", cycle_time_error_threshold_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time_);

  // Start timer
  ros::Duration desired_update_freq_ = ros::Duration(1 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(desired_update_freq_, &FiveD_HW_Loop::update, this);
}

void FiveD_HW_Loop::update(const ros::TimerEvent& e)
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
  
  ros::Duration desired_update_freq_ = ros::Duration(1 / loop_hz_);
  
  elapsed_time_ =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  last_time_ = current_time_;
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "FiveD_hw_main","Sampled update loop with elapsed time" << elapsed_time_.toSec());

  // Error check cycle time
  const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_)
  {
    ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time_
                                     << ", threshold: " << cycle_time_error_threshold_);
  }

  //ROS_WARN_STREAM_NAMED(name_, "Commands are: " << hardware_interface_->printCommandHelper() ) ;
  //ROS_WARN_STREAM_NAMED(name_, "States are: " << hardware_interface_->printStateHelper() ) ;
  // Input
  hardware_interface_->read(elapsed_time_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  hardware_interface_->write(elapsed_time_);
}


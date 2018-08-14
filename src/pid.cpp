#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

struct PidParams
{
  double kp;
  double ki;
  double kd;
  double clamp;
  bool angular;
};


class PID
{
public:
  PID()
  {
    params_ = {0, 0, 0, 0, false};
    e_sum_ = e_old_ = setpoint_ = 0;
    enabled_ = false;
  }

  PID(PidParams *p)
  {
    params_ = *p;
    e_sum_ = e_old_ = 0;
  }

  void setParams(PidParams *p)
  {
    params_ = *p;
  }

  void setSetpoint(double setpoint)
  {
    setpoint_ = setpoint;
  }

  void setState(double state)
  {
    state_ = state;
  }

  double getOutput()
  {
    return enabled_ ? output_ : 0;
  }

  void update()
  {
    double e = getError_();

    double p = params_.kp * e;
    double d = params_.kd * (e - e_old_);
    e_sum_ += params_.ki * e;

    if (abs(e_sum_) > params_.clamp)
      e_sum_ = copysign(params_.clamp, e_sum_);

    e_old_ = e;
    output_ = p + e_sum_ + d;
  }

  void enabled(bool e)
  {
    enabled_ = e;
  }

private:
  double getError_()
  {
    double e = setpoint_ - state_;
    if (params_.angular)
      e = atan2(sin(e), cos(e));
    return e;
  }
  PidParams params_;

  double setpoint_;
  double e_sum_;
  double e_old_;

  double state_;
  double output_;

  bool enabled_;
};

class PIDNode
{
public:
  PIDNode(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    setParams(pnh, heave_pid_, "heave");
    setParams(pnh, pitch_pid_, "pitch");

    wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("command", 1);

    heave_setpoint_sub_ = nh.subscribe<std_msgs::Float64>(
          "heave_pid/setpoint", 1, &PIDNode::heaveSetpointCallback, this);

    pitch_setpoint_sub_ = nh.subscribe<std_msgs::Float64>(
          "pitch_pid/setpoint", 1, &PIDNode::pitchSetpointCallback, this);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(
          "imu", 1, &PIDNode::imuCallback, this);

    pressure_sub_ = nh.subscribe<sensor_msgs::FluidPressure>(
          "pressure", 1, &PIDNode::pressureCallback, this);

    heave_service_ = nh.advertiseService("heave_pid/enabled",
                                         &PIDNode::heaveEnableCallback, this);
    pitch_service_ = nh.advertiseService("pitch_pid/enabled",
                                         &PIDNode::pitchEnableCallback, this);

  }

  void setParams(ros::NodeHandle& pnh, PID& pid, std::string name)
  {
    PidParams pid_params;
    pid_params.kp = pnh.param("pid/" + name + "/kp", 1.);
    pid_params.ki = pnh.param("pid/" + name + "/ki", 0.);
    pid_params.kd = pnh.param("pid/" + name + "/kd", 0.);
    pid_params.clamp = pnh.param("pid/" + name + "/clamp", 0.);
    pid_params.angular = pnh.param("pid/" + name + "/angular", false);
    pid.setParams(&pid_params);
  }

  bool heaveEnableCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
  {
    heave_pid_.enabled(req.data);
    return res.success = true;
  }

  bool pitchEnableCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
  {
    pitch_pid_.enabled(req.data);
    return res.success = true;
  }

  void heaveSetpointCallback(const std_msgs::Float64ConstPtr& msg)
  {
    heave_pid_.setSetpoint(msg->data);
  }

  void pitchSetpointCallback(const std_msgs::Float64ConstPtr& msg)
  {
    pitch_pid_.setSetpoint(msg->data);
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pitch_pid_.setState(pitch);
  }

  void pressureCallback(const sensor_msgs::FluidPressureConstPtr& msg)
  {
    heave_pid_.setState(msg->fluid_pressure);
  }

  void spin()
  {
    while (ros::ok())
    {
      pitch_pid_.update();
      heave_pid_.update();


      wrench_msg_.wrench.torque.y = pitch_pid_.getOutput();
      wrench_msg_.wrench.force.z = heave_pid_.getOutput();

      wrench_pub_.publish(wrench_msg_);
      ros::spinOnce();
      ros::Rate(10).sleep();
    }
  }

private:
  ros::Publisher wrench_pub_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pressure_sub_;

  ros::ServiceServer heave_service_;
  ros::ServiceServer pitch_service_;

  ros::Subscriber heave_setpoint_sub_;
  ros::Subscriber pitch_setpoint_sub_;

  PID heave_pid_;
  PID pitch_pid_;

  geometry_msgs::WrenchStamped wrench_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid");
  ros::NodeHandle nh;

  PIDNode p(nh);
  p.spin();
}

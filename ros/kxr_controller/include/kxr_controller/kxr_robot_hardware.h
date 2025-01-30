#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace kxr_controller {

class KXRRobotHW : public hardware_interface::RobotHW {
public:
    KXRRobotHW() : joint_state_received_(false) {};
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

    ros::Duration getPeriod() const { return control_loop_period_; }
    ros::Time getTime() const { return ros::Time::now(); }

    ros::NodeHandle nh_;

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_position_interface;
    hardware_interface::VelocityJointInterface joint_velocity_interface;

    std::map<std::string, std::string> mimic_joint_map_;
    std::map<std::string, double> mimic_joint_multiplier_;
    std::map<std::string, double> mimic_joint_offset_;

    ros::Duration control_loop_period_;
    bool joint_state_received_;
    std::map<std::string, unsigned int> jointname_to_id_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
    ros::Publisher joint_velocity_command_pub_;
    sensor_msgs::JointState current_joint_state_;
    sensor_msgs::JointState command_joint_state_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_state_position_;
    std::vector<double> joint_state_velocity_;
    std::vector<double> joint_state_effort_;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    boost::mutex mutex_;
};
}  // end of namespace kxr_controller

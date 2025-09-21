#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <memory>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define VehicleLength 3.5932
#define VehicleWidth 1.966
#define WheelRadius 0.497
#define P 11.7553507260245
#define I 0.473007565420235
#define D 64.9118618875423
#define N 788.228671066606
// #define P 10
// #define I 0.1
// #define D 0.5

namespace gazebo_plugins
{
class DrivePlugin : public gazebo::ModelPlugin
{
public:
DrivePlugin() : gazebo::ModelPlugin() {}


void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
{
    this->model_ = model;
    this->ros_node_ = gazebo_ros::Node::Get(sdf);

    RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Model name: %s", model->GetName().c_str());

    this->InitJoints(sdf);

    this->subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));

    RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /wheel_torque_command");

    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DrivePlugin::OnUpdate, this));
}



private:
void InitJoints(sdf::ElementPtr sdf)
{
    this->left_front_wheel_joint_ = model_->GetJoint("front_left_steer_joint");
    this->right_front_wheel_joint_ = model_->GetJoint("front_right_steer_joint");
    this->left_back_wheel_joint_ = model_->GetJoint("back_left_wheel_joint");
    this->right_back_wheel_joint_ = model_->GetJoint("back_right_wheel_joint");
    this->left_front_speed_wheel_joint_ = model_->GetJoint("front_left_wheel_joint");
    this->right_front_speed_wheel_joint_ = model_->GetJoint("front_right_wheel_joint");


    if (!this->left_back_wheel_joint_ || !this->right_back_wheel_joint_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Back Wheels joints not found");
        return;
    }

    if (!this->left_front_wheel_joint_ || !this->right_front_wheel_joint_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Wheels joints not found");
        return;
    }

    if (!this->left_front_speed_wheel_joint_ || !this->right_front_speed_wheel_joint_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Wheels joints not found");
        return;
    }


}

void OnUpdate()
{   
    //RCLCPP_INFO(ros_node_->get_logger(), "Current speed: %f, Steering: %f", desired_speed_, Steering_Request);
    //apply_steering(Steering_Request);
    // apply_efforts_suspension();
    //ApplyWheelSpeed(desired_speed_);
          // Control simple de los joints (es solo un ejemplo, ajusta según la lógica que necesites)
    left_front_wheel_joint_->SetPosition(0, Steering_Request);
    right_front_wheel_joint_->SetPosition(0, -Steering_Request);
    left_back_wheel_joint_->SetVelocity(0, desired_speed_);
    right_back_wheel_joint_->SetVelocity(0, -desired_speed_);
    left_front_speed_wheel_joint_->SetVelocity(0, desired_speed_);
    right_front_speed_wheel_joint_->SetVelocity(0, -desired_speed_);
}







void OnTorqueCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    //RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Received torque command: linear.x=%f, angular.z=%f",
    //            msg->linear.x, msg->angular.z);

    desired_speed_ = msg->linear.x;
    Steering_Request = msg->angular.z;
}

gazebo::physics::ModelPtr model_;
gazebo::physics::JointPtr left_front_wheel_joint_;
gazebo::physics::JointPtr right_front_wheel_joint_;
gazebo::physics::JointPtr left_back_wheel_joint_;
gazebo::physics::JointPtr right_back_wheel_joint_;
gazebo::physics::JointPtr left_front_speed_wheel_joint_;
gazebo::physics::JointPtr right_front_speed_wheel_joint_;
std::vector<gazebo::physics::JointPtr> suspension_joints_;
std::shared_ptr<gazebo_ros::Node> ros_node_;
gazebo::event::ConnectionPtr update_connection_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

ignition::math::PID wheel_pid_;
ignition::math::PID suspension_pid_;
double Steering_Request = 0.0;
double DesiredAngle = 0.0;
double DesiredAngleR = 0.0;
double IerL = 0.0;
double IerR = 0.0;
double steeringSpeed = 1.0;
double deltaSimTime = 0.001;
double left_velocity_ = 0.0;
double right_velocity_ = 0.0;
double desired_speed_ = 0.0; // Initial desired speed
double prevErrorL = 0.0;
double prevErrorR = 0.0;
double prev_error= 0.0;
double integral_error= 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(DrivePlugin)
} // namespace gazebo_plugins
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace gazebo {

class GazeboRosDiffDriveMultiWheel : public ModelPlugin {
public:
  GazeboRosDiffDriveMultiWheel() : ModelPlugin() {}

  ~GazeboRosDiffDriveMultiWheel() {
    // Los recursos se liberan automáticamente con los smart pointers.
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
    // Guardar punteros al modelo y al mundo
    this->parent = _parent;
    this->world = _parent->GetWorld();

    // Leer parámetros del SDF
    if (_sdf->HasElement("robotNamespace"))
      this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
    else
      this->robot_namespace_ = "";

    // Se esperan etiquetas <leftJoints> y <rightJoints> con nombres separados por espacios.
    // (Aquí se asume una sola entrada para simplificar; en una implementación completa se debe hacer un split de la cadena)
    if (_sdf->HasElement("leftJoints"))
      joint_names_left_.push_back(_sdf->Get<std::string>("leftJoints"));
    else {
      RCLCPP_ERROR(rclcpp::get_logger("GazeboRosDiffDriveMultiWheel"), "Falta <leftJoints> en SDF");
      return;
    }

    if (_sdf->HasElement("rightJoints"))
      joint_names_right_.push_back(_sdf->Get<std::string>("rightJoints"));
    else {
      RCLCPP_ERROR(rclcpp::get_logger("GazeboRosDiffDriveMultiWheel"), "Falta <rightJoints> en SDF");
      return;
    }

    // Otros parámetros
    wheel_separation_ = _sdf->HasElement("wheelSeparation") ? _sdf->Get<double>("wheelSeparation") : 0.34;
    wheel_diameter_   = _sdf->HasElement("wheelDiameter")   ? _sdf->Get<double>("wheelDiameter")   : 0.15;
    torque            = _sdf->HasElement("torque")            ? _sdf->Get<double>("torque")            : 5.0;
    command_topic_    = _sdf->HasElement("commandTopic")      ? _sdf->Get<std::string>("commandTopic")   : "cmd_vel";
    odometry_topic_   = _sdf->HasElement("odometryTopic")     ? _sdf->Get<std::string>("odometryTopic")  : "odom";
    odometry_frame_   = _sdf->HasElement("odometryFrame")     ? _sdf->Get<std::string>("odometryFrame")  : "odom";
    robot_base_frame_ = _sdf->HasElement("robotBaseFrame")    ? _sdf->Get<std::string>("robotBaseFrame") : "base_footprint";
    update_rate_      = _sdf->HasElement("updateRate")        ? _sdf->Get<double>("updateRate")          : 100.0;

    update_period_ = (update_rate_ > 0.0) ? 1.0 / update_rate_ : 0.0;
    last_update_time_ = this->world->SimTime();

    // Obtener punteros a los joints
    for (auto &name : joint_names_left_) {
      auto joint = _parent->GetJoint(name);
      if (!joint) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosDiffDriveMultiWheel"), "No se pudo obtener el joint izquierdo: %s", name.c_str());
      } else {
        joints_left_.push_back(joint);
        joint->SetEffortLimit(0, torque);
      }
    }
    for (auto &name : joint_names_right_) {
      auto joint = _parent->GetJoint(name);
      if (!joint) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosDiffDriveMultiWheel"), "No se pudo obtener el joint derecho: %s", name.c_str());
      } else {
        joints_right_.push_back(joint);
        joint->SetEffortLimit(0, torque);
      }
    }

    // Inicializar el nodo ROS2 usando gazebo_ros::Node (ya configurado para ROS2)
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(ros_node_->get_logger(), "Iniciando GazeboRosDiffDriveMultiWheel Plugin (ns = %s)", this->robot_namespace_.c_str());

    // Crear suscripción al tópico de comando de velocidad
    cmd_vel_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 1,
      std::bind(&GazeboRosDiffDriveMultiWheel::cmdVelCallback, this, std::placeholders::_1)
    );

    // Crear publicador para la odometría
    odometry_publisher_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 1);

    // Crear transform broadcaster para publicar TF usando tf2
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    // Conectar al evento de actualización del mundo de Gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDiffDriveMultiWheel::UpdateChild, this)
    );
  }

  // Función que se llama en cada ciclo de simulación
  void UpdateChild() {
    auto current_time = this->world->SimTime();
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {
      publishOdometry(seconds_since_last_update);
      getWheelVelocities();

      // Asignar velocidad a cada joint
      for (auto &joint : joints_left_) {
        joint->SetVelocity(0, wheel_speed_left_ / (0.5 * wheel_diameter_));
      }
      for (auto &joint : joints_right_) {
        joint->SetVelocity(0, wheel_speed_right_ / (0.5 * wheel_diameter_));
      }

      last_update_time_ += common::Time(update_period_);
    }
  }

  void getWheelVelocities() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Calcula las velocidades de las ruedas en función de x_ y rot_
    wheel_speed_left_ = x_ - rot_ * wheel_separation_ / 2.0;
    wheel_speed_right_ = x_ + rot_ * wheel_separation_ / 2.0;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  void publishOdometry(double step_time) {
    // Obtener tiempo simulado y convertir a rclcpp::Time
    auto sim_time = this->world->SimTime();
    rclcpp::Time current_time(sim_time.sec, sim_time.nsec, rclcpp::Clock(RCL_ROS_TIME));

    // Obtener la pose del robot en el mundo
    ignition::math::Pose3d pose = this->parent->WorldPose();

    // Publicar el transform TF
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = odometry_frame_;
    odom_tf.child_frame_id = robot_base_frame_;
    odom_tf.transform.translation.x = pose.Pos().X();
    odom_tf.transform.translation.y = pose.Pos().Y();
    odom_tf.transform.translation.z = pose.Pos().Z();
    tf2::Quaternion q;
    q.setRPY(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    transform_broadcaster_->sendTransform(odom_tf);

    // Construir y publicar el mensaje de odometría
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Aquí se puede rellenar la covarianza si es necesario

    // Obtener la velocidad lineal en el marco del mundo
    ignition::math::Vector3d linear = this->parent->WorldLinearVel();
    double yaw = pose.Rot().Yaw();
    odom.twist.twist.linear.x = cos(yaw) * linear.X() + sin(yaw) * linear.Y();
    odom.twist.twist.linear.y = cos(yaw) * linear.Y() - sin(yaw) * linear.X();
    odom.twist.twist.angular.z = this->parent->WorldAngularVel().Z();

    odometry_publisher_->publish(odom);
  }

private:
  // Punteros y variables miembro
  physics::ModelPtr parent;
  physics::WorldPtr world;
  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  double wheel_separation_;
  double wheel_diameter_;
  double torque;
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;

  std::vector<std::string> joint_names_left_;
  std::vector<std::string> joint_names_right_;
  std::vector<physics::JointPtr> joints_left_;
  std::vector<physics::JointPtr> joints_right_;

  // Variables para el estado del robot
  double x_ = 0.0;
  double rot_ = 0.0;
  double wheel_speed_left_ = 0.0;
  double wheel_speed_right_ = 0.0;

  // Nodo ROS2, suscriptor, publicador y transform broadcaster
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::mutex mutex_;

  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDriveMultiWheel)

} // namespace gazebo

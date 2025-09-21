#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo {

class GazeboRosDiffDriveMultiWheel : public ModelPlugin
{
public:
  GazeboRosDiffDriveMultiWheel() : ModelPlugin(), alive_(true), wheel_angle_(0.0) {}
  virtual ~GazeboRosDiffDriveMultiWheel();

  // Se llama al cargar el plugin
  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  // Función de actualización conectada al evento de Gazebo
  void UpdateChild();
  // Función de finalización del plugin
  void FiniChild();

protected:
  // Callback para cmd_vel
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // Función que ejecuta el spin del nodo ROS2 en un hilo
  void QueueThread();
  // Publica la odometría (y TF, incluyendo los del Lidar y ruedas) si se ha pedido
  void publishOdometry(double step_time);
  // Calcula la velocidad de cada rueda según el modelo diferencial
  void getWheelVelocities();

  // Variables de ROS2
  std::shared_ptr<rclcpp::Node> rosnode_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::thread queue_thread_;
  std::mutex lock_;
  bool alive_;

  // Parámetros y nombres
  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  double update_rate_;
  double update_period_;
  bool publish_odometry_tf_;
  bool publish_odometry_msg_;

  // Parámetros del robot
  double wheel_separation_;
  double wheel_diameter_;
  double torque;

  // Parámetros para el Lidar (transformación fija de base_link a Lidar)
  std::string lidar_link_ = "lidar_link";
  double lidar_offset_x_ = 0.2;
  double lidar_offset_y_ = 0.0;
  double lidar_offset_z_ = 0.1;
  // Se asume orientación identidad para el Lidar

  // Estado interno para cmd_vel
  double x_, rot_;
  // Velocidades calculadas para cada lado [RIGHT, LEFT]
  enum { RIGHT, LEFT };
  double wheel_speed_[2];

  // Para las ruedas (acumulación de giro de la llanta)
  double wheel_angle_; // Se debe actualizar según la integración; aquí se inicia en 0.0

  // Nombres y punteros a joints para cada lado
  std::vector<std::string> joint_names_[2];
  std::vector<physics::JointPtr> joints_[2];

  // Punteros a la simulación
  physics::ModelPtr parent;
  physics::WorldPtr world;
  common::Time last_update_time_;

  // Mensaje de odometría (para publicación)
  nav_msgs::msg::Odometry odom_;

  // Conexión al evento de actualización de Gazebo
  event::ConnectionPtr update_connection_;
};

/////////////////////////////////////////////////
GazeboRosDiffDriveMultiWheel::~GazeboRosDiffDriveMultiWheel()
{
  alive_ = false;
  if (queue_thread_.joinable())
    queue_thread_.join();
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Guardar punteros a la simulación
  this->parent = _parent;
  this->world  = _parent->GetWorld();

  // Leer namespace (opcional)
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
  else
    this->robot_namespace_ = "";

  // Leer nombres de joints de las ruedas izquierdas y derechas
  if (!_sdf->HasElement("leftJoints"))
  {
    gzthrow("GazeboRosDiffDriveMultiWheel Plugin: Falta especificar <leftJoints>.");
  }
  else {
    std::string joint_string = _sdf->Get<std::string>("leftJoints");
    std::istringstream iss(joint_string);
    std::string joint;
    while (iss >> joint)
      joint_names_[LEFT].push_back(joint);
  }
  if (!_sdf->HasElement("rightJoints"))
  {
    gzthrow("GazeboRosDiffDriveMultiWheel Plugin: Falta especificar <rightJoints>.");
  }
  else {
    std::string joint_string = _sdf->Get<std::string>("rightJoints");
    std::istringstream iss(joint_string);
    std::string joint;
    while (iss >> joint)
      joint_names_[RIGHT].push_back(joint);
  }

  // Leer parámetros del robot
  if (_sdf->HasElement("wheelSeparation"))
    this->wheel_separation_ = _sdf->Get<double>("wheelSeparation");
  else
    this->wheel_separation_ = 0.34;

  if (_sdf->HasElement("wheelDiameter"))
    this->wheel_diameter_ = _sdf->Get<double>("wheelDiameter");
  else
    this->wheel_diameter_ = 0.15;

  if (_sdf->HasElement("torque"))
    this->torque = _sdf->Get<double>("torque");
  else
    this->torque = 5.0;

  // Leer tópicos y frames
  if (_sdf->HasElement("commandTopic"))
    this->command_topic_ = _sdf->Get<std::string>("commandTopic");
  else
    this->command_topic_ = "cmd_vel";

  if (_sdf->HasElement("odometryTopic"))
    this->odometry_topic_ = _sdf->Get<std::string>("odometryTopic");
  else
    this->odometry_topic_ = "odom";

  if (_sdf->HasElement("odometryFrame"))
    this->odometry_frame_ = _sdf->Get<std::string>("odometryFrame");
  else
    this->odometry_frame_ = "odom";

  if (_sdf->HasElement("robotBaseFrame"))
    this->robot_base_frame_ = _sdf->Get<std::string>("robotBaseFrame");
  else
    this->robot_base_frame_ = "base_link";

  if (_sdf->HasElement("updateRate"))
    this->update_rate_ = _sdf->Get<double>("updateRate");
  else
    this->update_rate_ = 100.0;

  if (this->update_rate_ > 0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  if (_sdf->HasElement("publishOdometryTf"))
    this->publish_odometry_tf_ = _sdf->Get<bool>("publishOdometryTf");
  else
    this->publish_odometry_tf_ = true;

  if (_sdf->HasElement("publishOdometryMsg"))
    this->publish_odometry_msg_ = _sdf->Get<bool>("publishOdometryMsg");
  else
    this->publish_odometry_msg_ = true;

  // Leer parámetros para el Lidar (si se especifica)
  if (_sdf->HasElement("lidarLink"))
    this->lidar_link_ = _sdf->Get<std::string>("lidarLink");
  if (_sdf->HasElement("lidarPose"))
  {
    sdf::ElementPtr lidarPoseElem = _sdf->GetElement("lidarPose");
    if (lidarPoseElem->HasElement("x"))
      this->lidar_offset_x_ = lidarPoseElem->Get<double>("x");
    if (lidarPoseElem->HasElement("y"))
      this->lidar_offset_y_ = lidarPoseElem->Get<double>("y");
    if (lidarPoseElem->HasElement("z"))
      this->lidar_offset_z_ = lidarPoseElem->Get<double>("z");
    // Se puede agregar orientación si es necesario (en este ejemplo se usa identidad)
  }

  // Inicializar velocidades y estado
  wheel_speed_[RIGHT] = 0;
  wheel_speed_[LEFT]  = 0;
  x_ = 0;
  rot_ = 0;
  wheel_angle_ = 0.0; // Inicialmente 0; debe actualizarse según integración real

  // Cargar punteros a los joints
  for (size_t side = 0; side < 2; ++side) {
    for (auto &joint_name : joint_names_[side]) {
      auto joint = this->parent->GetJoint(joint_name);
      if (!joint)
      {
        std::string error = "GazeboRosDiffDriveMultiWheel Plugin: No se encontró el joint " + joint_name;
        gzthrow(error);
      }
      joints_[side].push_back(joint);
      joint->SetEffortLimit(0, torque);
    }
  }

  // Crear nodo ROS2 (interno al plugin)
  rosnode_ = rclcpp::Node::make_shared("gazebo_ros_diff_drive_multi_wheel");
  RCLCPP_INFO(rosnode_->get_logger(), "Iniciando GazeboRosDiffDriveMultiWheel Plugin (ns = %s)", robot_namespace_.c_str());

  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(rosnode_);

  cmd_vel_subscriber_ = rosnode_->create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&GazeboRosDiffDriveMultiWheel::cmdVelCallback, this, std::placeholders::_1));

  odometry_publisher_ = rosnode_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);

  queue_thread_ = std::thread(std::bind(&GazeboRosDiffDriveMultiWheel::QueueThread, this));

  this->last_update_time_ = this->world->SimTime();
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDiffDriveMultiWheel::UpdateChild, this));
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::UpdateChild()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_)
  {
    if (publish_odometry_tf_ || publish_odometry_msg_)
      publishOdometry(seconds_since_last_update);

    getWheelVelocities();

    // Actualizar la velocidad de cada joint
    for (size_t side = 0; side < 2; ++side)
    {
      for (size_t i = 0; i < joints_[side].size(); ++i)
      {
        joints_[side][i]->SetVelocity(0, wheel_speed_[side] / (0.5 * wheel_diameter_));
      }
    }
    last_update_time_ += common::Time(update_period_);
  }
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::FiniChild()
{
  alive_ = false;
  if (queue_thread_.joinable())
    queue_thread_.join();
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::getWheelVelocities()
{
  std::lock_guard<std::mutex> guard(lock_);
  double vr = rot_;
  double va = x_;
  wheel_speed_[LEFT]  = vr - va * wheel_separation_ / 2.0;
  wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(lock_);
  x_ = -msg->linear.x;
  rot_ = -msg->angular.z;
  // Aquí, si se desea, se puede integrar para actualizar wheel_angle_ (por ejemplo, usando: delta_distance / wheel_radius)
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::QueueThread()
{
  rclcpp::Rate rate(100);
  while (alive_ && rclcpp::ok())
  {
    rclcpp::spin_some(rosnode_);
    rate.sleep();
  }
}

/////////////////////////////////////////////////
void GazeboRosDiffDriveMultiWheel::publishOdometry(double step_time)
{
  
  gazebo::common::Time current_time = this->world->SimTime();

  // Convertir a builtin_interfaces::msg::Time
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = current_time.sec;
  ros_time.nanosec = current_time.nsec;

  // Obtener la pose del robot (T_odom_base)
  ignition::math::Pose3d pose = this->parent->WorldPose();

  // Publicar TF para la base: odom -> base_link
  geometry_msgs::msg::TransformStamped base_transform;
  base_transform.header.stamp = ros_time;
  base_transform.header.frame_id = odometry_frame_;
  base_transform.child_frame_id = robot_base_frame_;
  base_transform.transform.translation.x = pose.Pos().X();
  base_transform.transform.translation.y = pose.Pos().Y();
  base_transform.transform.translation.z = pose.Pos().Z();
  auto quat = pose.Rot();
  base_transform.transform.rotation.x = quat.X();
  base_transform.transform.rotation.y = quat.Y();
  base_transform.transform.rotation.z = quat.Z();
  base_transform.transform.rotation.w = quat.W();
  if (publish_odometry_tf_)
    transform_broadcaster_->sendTransform(base_transform);

  // // Transformación fija del Lidar: T_base_lidar
  // ignition::math::Pose3d T_base_lidar(lidar_offset_x_, lidar_offset_y_, lidar_offset_z_, 0, 0, 0);
  // // Componer: T_odom_lidar = T_odom_base * T_base_lidar
  // ignition::math::Pose3d T_odom_lidar = pose * T_base_lidar;
  // geometry_msgs::msg::TransformStamped lidar_transform;
  // lidar_transform.header.stamp = current_time;
  // lidar_transform.header.frame_id = robot_base_frame_; // Para mantener la cadena global
  // lidar_transform.child_frame_id = lidar_link_;
  // lidar_transform.transform.translation.x = T_odom_lidar.Pos().X();
  // lidar_transform.transform.translation.y = T_odom_lidar.Pos().Y();
  // lidar_transform.transform.translation.z = T_odom_lidar.Pos().Z();
  // auto lidar_quat = T_odom_lidar.Rot();
  // lidar_transform.transform.rotation.x = lidar_quat.X();
  // lidar_transform.transform.rotation.y = lidar_quat.Y();
  // lidar_transform.transform.rotation.z = lidar_quat.Z();
  // lidar_transform.transform.rotation.w = lidar_quat.W();
  // if (publish_odometry_tf_)
  //   transform_broadcaster_->sendTransform(lidar_transform);

  // Publicar el mensaje de odometría (usando la pose del robot)
  odom_.header.stamp = ros_time;
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.pose.pose.position.x = pose.Pos().X();
  odom_.pose.pose.position.y = pose.Pos().Y();
  odom_.pose.pose.position.z = pose.Pos().Z();
  odom_.pose.pose.orientation.x = quat.X();
  odom_.pose.pose.orientation.y = quat.Y();
  odom_.pose.pose.orientation.z = quat.Z();
  odom_.pose.pose.orientation.w = quat.W();
  odom_.pose.covariance[0]  = 1e-5;
  odom_.pose.covariance[7]  = 1e-5;
  odom_.pose.covariance[14] = 1e12;
  odom_.pose.covariance[21] = 1e12;
  odom_.pose.covariance[28] = 1e12;
  odom_.pose.covariance[35] = 0.001;
  ignition::math::Vector3d linear = this->parent->WorldLinearVel();
  double angular_z = this->parent->WorldAngularVel().Z();
  double yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cos(yaw) * linear.X() + sin(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cos(yaw) * linear.Y() - sin(yaw) * linear.X();
  odom_.twist.twist.angular.z = angular_z;
  if (publish_odometry_msg_)
    odometry_publisher_->publish(odom_);


}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDriveMultiWheel)
} // namespace gazebo

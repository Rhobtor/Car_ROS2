#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>

namespace gazebo_plugins
{
  class JointStatePublisherPlugin : public gazebo::ModelPlugin
  {
  public:
    JointStatePublisherPlugin() : ModelPlugin() {}

    // Se llama al cargar el plugin
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Guardamos el puntero al modelo
      this->model_ = _model;
      // Obtenemos el nodo ROS desde gazebo_ros
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      // Creamos el publicador para sensor_msgs/JointState
      this->joint_state_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

      // Nos conectamos al evento de actualización del mundo
      this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointStatePublisherPlugin::OnUpdate, this));

      RCLCPP_INFO(ros_node_->get_logger(), "JointStatePublisherPlugin loaded for model: %s", _model->GetName().c_str());
    }

    // Esta función se llama en cada ciclo de simulación
    void OnUpdate()
    {
      // Creamos el mensaje de JointState
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = this->ros_node_->now();

      // Obtenemos la lista de joints del modelo
      std::vector<gazebo::physics::JointPtr> joints = this->model_->GetJoints();
      for (auto &joint : joints)
      {
        // Se añaden los nombres de los joints
        joint_state_msg.name.push_back(joint->GetName());
        // Se añade la posición (para joints rotacionales se usa Position(0))
        joint_state_msg.position.push_back(joint->Position(0));
        // Se añade la velocidad (si está disponible)
        joint_state_msg.velocity.push_back(joint->GetVelocity(0));
        // Se añade el esfuerzo o torque (opcional)
        joint_state_msg.effort.push_back(joint->GetForce(0));
      }

      // Publicamos el mensaje en el tópico /joint_states
      this->joint_state_pub_->publish(joint_state_msg);
    }

  private:
    gazebo::physics::ModelPtr model_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    gazebo::event::ConnectionPtr update_connection_;
  };

  // Registro del plugin con Gazebo
  GZ_REGISTER_MODEL_PLUGIN(JointStatePublisherPlugin)
}  // namespace gazebo_plugins

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "octomap/AbstractOcTree.h"
#include <fstream>
#include "obstacle_detection.hpp"
#include "odometria.hpp"
#include "utilities.hpp"
#include "motion.hpp"
#include "visualization.hpp"

class move_ros : public rclcpp::Node {
private:
    // Objetos de clases utilizadas
    obstacle_detection obstacle_detected_p;
    odometria odometry_p;
    motion motion_p;
    visualization visualization_p;
    utilities utilities_p;

    // Subscripciones y publicadores
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

    // Variables relacionadas con OctoMap y odometría
    octomap::AbstractOcTree* tree;
    octomap::OcTree* octree;

    nav_msgs::msg::Odometry::ConstPtr odometry;

    // Variables de estado y configuración
    std::vector<double> pos_actual;
    std::vector<double> orientacion_actual_q;
    double orientacion_actual_E;
    double orientacion_prueba;
    double distancia_final;
    char nueva_posicion_final;
    char continuar;
    bool bool_odom;
    bool octomap_recibido;
    double distancia_punto_destino;
    double distancia_anterior_punto_destino;
    double angulo_punto_destino;
    double dif_angular;
    char cont;
    int interaciones_ejecucion_totales;
    double radio_circunferencia;
    double numPuntos_circunferencia;
    bool evaluar_recta;
    bool punto_nuevo;
    bool no_punto_valido;

    // Variables específicas del algoritmo
    double radio_ext;
    double radio_int;
    double num_puntos;
    double distancia_calculo;
    double tolerancia_angular;
    double tolerancia_distancia;
    double tolerancia_distancia_final;
    octomap::point3d PuntoFIN3d;
    double pos_fin_x;
    double pos_fin_y;
    double pos_fin_z;

    std::vector<octomap::point3d> Vector_puntos_aleatorios;
    std::vector<double> Vector_distancias_pto_final;
    std::vector<octomap::point3d> vector_intermediatePoints;
    std::vector<octomap::point3d> vector_circunferencePoints;

    octomap::point3d pos_actual3d;
    std::vector<double> RandomPoint;
    octomap::point3d RandomPoint3d;
    bool occupation;
    bool ocupacion_punto_destino;
    bool inter_occupation;
    bool circunference_occupation;
    double distancia_pto_final;
    int indice_min_distancia;
    octomap::point3d pos_destino3d;
    geometry_msgs::msg::Twist Comando_velocidad;
    bool no_avanzar;

    visualization_msgs::msg::Marker borrar_puntos;
    visualization_msgs::msg::Marker punto_visualizacion;
    visualization_msgs::msg::MarkerArray array_visualizacion;
    visualization_msgs::msg::Marker linea_visualizacion;

    // Callbacks
    void octomapCallback(const octomap_msgs::msg::Octomap::ConstPtr& octomap_msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstPtr& odometry_);

public:
    move_ros();
    ~move_ros();

    void run();
};
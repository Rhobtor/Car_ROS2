#include "iostream" 
#include "cmath" 
#include "cstdlib" 
#include "ctime" 
#include "vector" 
#include "octomap_msgs/msg/octomap.h" 
#include "octomap_msgs/conversions.h" 
#include "octomap/OcTree.h" 
 
class utilities 
{ 
private: 
    std::vector<double> vector; 
    octomap::point3d octoPoint3d; 
 
    double min_theta; 
    double max_theta; 
    double theta; 
    double distance; 
    double x_centro; 
    double y_centro; 
    double z_centro; 
    std::vector<double> randomPoint; 
    double diferenciaX; 
    double diferenciaY; 
    double diferenciaZ; 
    double distancia; 
    std::vector<double> intermediatePoint; 
    octomap::point3d intermediatePoint3d; 
    std::vector<octomap::point3d> intermediateVectorPoints3d; 
    octomap::point3d puntoInicio3d; 
    octomap::point3d puntoFin3d; 
    double distanciaTotal; 
    int cantidadPuntos; 
    double angulo_circunferencia; 
    octomap::point3d punto_circunferencia; 
    std::vector<octomap::point3d> vector_puntos_circunferencia; 
    std::vector<double> vectorDistances; 
    double minimumDistance; 
    int minimumIndex; 
    octomap::point3d FinalPoint3d; 
    octomap::point3d StartPoint3d; 
    double deltaX; 
    double deltaY;
     double hipotenusa; 
    double angle; 
    double Euler_Angle; 
    double qx; 
    double qy; 
    double qz; 
    double qw; 
     
public: 
    utilities(); 
    ~ utilities(); 
 
    octomap::point3d VectorToPoint3d (std::vector<double> vector); 
    std::vector<double> Point3dToVector(octomap::point3d octoPoint3d); 
    std::vector<double> generateRandomPoint(std::vector<double> centro, double 
orientacion, double radio_ext, double radio_int); 
    double calculateDistance(octomap::point3d puntoA, octomap::point3d puntoB); 
    std::vector<octomap::point3d> createVectorIntermediatePoints3d(octomap::point3d 
puntoInicio3d, octomap::point3d puntoFin3d, double incremento); 
    std::vector<octomap::point3d> createCircunferencePoints3d(octomap::point3d 
centro, double radio, int numPuntos); 
    int minimumValueIndex(std::vector<double> vectorDistances); 
    double calculateAngle(octomap::point3d StartPoint3d, octomap::point3d 
FinalPoint3d); 
    double CuaternionToEulerAngles (std::vector<double> cuaternion_orientation); 
 
}; 

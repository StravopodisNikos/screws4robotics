#include "screws4robotics.h"

namespace screws_math_ns {

screws4robotics::screws4robotics()
{
    // Constructor implementation here, if needed
}

screws4robotics::~screws4robotics()
{
    // Destructor implementation here, if needed
}

Eigen::Vector3f screws4robotics::crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    Eigen::Vector3f result;
    result << v1.y() * v2.z() - v1.z() * v2.y(),
            v1.z() * v2.x() - v1.x() * v2.z(),
            v1.x() * v2.y() - v1.y() * v2.x();
    return result;
}

void screws4robotics::printVector(Stream& serialPort, const Eigen::Vector3f& v) {
  serialPort.print("Vector: [");
  serialPort.print(v[0]);
  serialPort.print(" ");
  serialPort.print(v[1]);
  serialPort.print(" ");
  serialPort.print(v[2]);
  serialPort.println("]");
}

void screws4robotics::formTwist( Eigen::Matrix<float, 6, 1> & T, const Eigen::Vector3f& v, const Eigen::Vector3f& w) {
    T << v , w;
}

void screws4robotics::printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> & T ) {
  serialPort.print("Twist: [");
  serialPort.print(T[0]);
  serialPort.print(" ");
  serialPort.print(T[1]);
  serialPort.print(" ");
  serialPort.print(T[2]);
  serialPort.print(" ");
  serialPort.print(T[3]);
  serialPort.print(" ");
  serialPort.print(T[4]);
  serialPort.print(" ");
  serialPort.print(T[5]);      
  serialPort.println("]");
}

} // namespace screws_math_ns

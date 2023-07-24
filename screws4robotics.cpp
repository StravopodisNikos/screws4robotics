#include "screws4robotics.h"

namespace screws_math_ns {

screws4robotics::screws4robotics()
{
    _st = 0;
    _ct = 1;
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

Eigen::Matrix3f screws4robotics::skew(const Eigen::Vector3f& w) {
    Eigen::Matrix3f m;
    m <<    0, -w(2),  w(1),
         w(2),     0, -w(0),
        -w(1),  w(0),     0;
    return m;    
}

Eigen::Matrix3f screws4robotics::skewExp(const Eigen::Vector3f& w, float theta) {
    // Since 3x1 vector is provided, must be transformed to 3x3 skew-symmetric matrix
    Eigen::Matrix3f R = skew(w);

    // theta must be 1x1 float value
    _st = std::sin(theta);
    _ct = std::cos(theta);
    Eigen::Matrix3f exp_w_theta = Eigen::Matrix3f::Identity() + R * _st + (R * R) * (1 - _ct);
    
    return exp_w_theta;
}

Eigen::Isometry3f screws4robotics::twistExp(Eigen::Matrix<float, 6, 1> & xi, float theta) {
    Eigen::Vector3f v = xi.segment(0, 3);
    Eigen::Vector3f w = xi.segment(3, 3);

    Eigen::Isometry3f g = Eigen::Isometry3f::Identity();

    if ( w.isZero() ) {
        // Pure translation case
        g.translation() = v * theta;
    } else {
        // Rotation and translation case
        Eigen::Matrix3f e = skewExp(w, theta);
        g.linear() = e;
        g.translation() = (Eigen::Matrix3f::Identity() - e) * (skew(w) * v) + w * w.transpose() * v * theta;
    }

    return g;
}

void screws4robotics::printTransform(Stream& serialPort, const Eigen::Isometry3f& tf) {
    serialPort.println("Transformation Matrix:");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            serialPort.print(tf(i, j),4); serialPort.print(" ");
        }
        serialPort.println();
    }
}

void screws4robotics::ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g ) {
// g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    A.block<3, 3>(0, 0) = R;
    A.block<3, 3>(0, 3) = pHat * R;
    A.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();
    A.block<3, 3>(3, 3) = R;
}

void screws4robotics::iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g ) {
// g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    A.block<3, 3>(0, 0) =  R.transpose();
    A.block<3, 3>(0, 3) = -R.transpose() * pHat;
    A.block<3, 3>(3, 0) =  Eigen::Matrix3f::Zero();
    A.block<3, 3>(3, 3) =  R.transpose();
}

template <int Rows, int Cols>
void screws4robotics::printMatrix(Eigen::Matrix<float, Rows, Cols>& matrix) {
    for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
            Serial.print(matrix(i, j),4);
            Serial.print("\t"); // Add a tab to separate the elements
        }
        Serial.println(); // Move to the next row
    }
}
template void screws4robotics::printMatrix<6,6>(Eigen::Matrix<float, 6, 6>&);

} // namespace screws_math_ns

#include "screws4robotics.h"

namespace screws_utils_ns {

screws4robotics::screws4robotics()
{
    _st = 0;
    _ct = 1;
}

screws4robotics::~screws4robotics()
{
    // Destructor implementation here, if needed
}

/*
 *  Screw Theory utilities
 */

Eigen::Vector3f screws4robotics::crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    Eigen::Vector3f result;
    result << v1.y() * v2.z() - v1.z() * v2.y(),
            v1.z() * v2.x() - v1.x() * v2.z(),
            v1.x() * v2.y() - v1.y() * v2.x();
    return result;
}

void screws4robotics::formTwist(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f v, Eigen::Vector3f w) {
    xi_R6 << v , w;
}

void screws4robotics::formTwist(Eigen::Matrix4f & xi_se3, Eigen::Vector3f v, Eigen::Matrix3f wHat) {
    xi_se3.block<3, 3>(0, 0) =  wHat;
    xi_se3.block<3, 1>(0, 3) =  v;
    xi_se3.block<1, 4>(3, 0) =  Eigen::Vector4f::Zero();
}

void screws4robotics::splitTwist(const Eigen::Matrix<float, 6, 1> xi_R6, Eigen::Vector3f & v, Eigen::Vector3f & w) {
    v = xi_R6.block<3, 1>(0, 0);
    w = xi_R6.block<3, 1>(3, 0);
}

void screws4robotics::splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Vector3f & w) {
    Eigen::Matrix<float, 6, 1> xi_R6;
    vee(xi_R6, xi_se3);
    v = xi_R6.block<3, 1>(0, 0);
    w = xi_R6.block<3, 1>(3, 0);    
}

void screws4robotics::splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Matrix3f & wHat) {
    wHat = xi_se3.block<3, 3>(0, 0);
    v    = xi_se3.block<3, 1>(0, 3); 
}



Eigen::Matrix3f screws4robotics::skew(const Eigen::Vector3f& w) {
    // w \in R3 -> wHat \in so(3)
    Eigen::Matrix3f m;
    m <<     0, -w.z(),  w.y(),
         w.z(),      0, -w.x(),
        -w.y(),  w.x(),      0;
    return m;    
}

Eigen::Vector3f screws4robotics::unskew(const Eigen::Matrix3f& wHat) {
    // wHat \in so(3) -> w \in R3
    Eigen::Vector3f w;
    w.x() = -wHat(1,2);
    w.y() =  wHat(0,2);
    w.z() =  wHat(1,0);
}

void screws4robotics::vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3) {
    // forms a 6x1 vector twist from a se(3) twist marix
    Eigen::Matrix3f wHat = xi_se3.block<3, 3>(0, 0);
    formTwist( xi, xi_se3.block<3, 1>(0, 3), unskew(wHat));
}

void screws4robotics::wedge(Eigen::Matrix4f& xi , Eigen::Matrix<float, 6, 1> xi_R6) {
    // forms a se(3) twist marix from a 6x1 vector twist
    Eigen::Vector3f v = xi_R6.segment(0, 3);
    Eigen::Vector3f w = xi_R6.segment(3, 3);
    Eigen::Matrix3f wHat = skew(w);
    formTwist( xi,v,wHat);
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

// Overloaded twistExp function for 4x4 twist R(6)
Eigen::Isometry3f screws4robotics::twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta) {
    Eigen::Isometry3f g = Eigen::Isometry3f::Identity();
    Eigen::Vector3f v = xi.block<3, 1>(0, 0);
    Eigen::Vector3f w = xi.block<3, 1>(3, 0);

    if (w.isZero()) {
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

// Overloaded twistExp function for 4x4 twist se(3)
Eigen::Isometry3f screws4robotics::twistExp(const Eigen::Matrix4f& xi, float theta) {
    Eigen::Matrix<float, 6, 1> xi_R6;
    vee(xi_R6, xi);
    Eigen::Vector3f v = xi_R6.block<3, 1>(0, 0);
    Eigen::Vector3f w = xi_R6.block<3, 1>(3, 0);

    Eigen::Isometry3f g = Eigen::Isometry3f::Identity();

    if (w.isZero()) {
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

void screws4robotics::ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g ) {
    // Described in [1]/p.55
    // g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    A.block<3, 3>(0, 0) = R;
    A.block<3, 3>(0, 3) = pHat * R;
    A.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();
    A.block<3, 3>(3, 3) = R;
}

void screws4robotics::Ad(Eigen::Matrix<float, 6, 6> & Ad, const Eigen::Isometry3f& Ci ) {
    // Described in [3]/p.241/eq.(96)
    // Ci \in SE(3) homogeneous rigid body tf wrt the base frame {S}(or {IFR} in Mueller's terms)
    Eigen::Matrix3f Ri = Ci.linear();
    Eigen::Vector3f ri = Ci.translation();

    Eigen::Matrix3f riHat = skew(ri);

    Ad.block<3, 3>(0, 0) = Ri;
    Ad.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
    Ad.block<3, 3>(3, 0) = riHat * Ri;
    Ad.block<3, 3>(3, 3) = Ri;
}

/*
Eigen::Matrix<float, 6, 6>  screws4robotics::adBold(Eigen::Matrix<float, 6, 1> xi_R6) {
    Eigen::Matrix<float, 6, 6> AD;

    AD.row(0) <<  0,       -xi_R6(5), xi_R6(4), 0,        -xi_R6(2),  xi_R6(1);
    AD.row(1) <<  xi_R6(5), 0,       -xi_R6(3), xi_R6(2),  0,        -xi_R6(0);
    AD.row(2) <<  xi_R6(4), xi_R6(3), 0,       -xi_R6(1),  xi_R6(0),  0;
    AD.row(3) <<  0,        0,        0,        0,        -xi_R6(5),  xi_R6(4);
    AD.row(4) <<  0,        0,        0,        xi_R6(5),  0,        -xi_R6(3);
    AD.row(5) <<  0,        0,        0,       -xi_R6(4),  xi_R6(3),  0;
    return AD;
}
*/

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

Eigen::Matrix4f  screws4robotics::lb(Eigen::Matrix4f xi_i_se3, Eigen::Matrix4f xi_j_se3) {
    // Lie bracket operator on se(3), as in [1]/p.175
    // Screw Product of twists \in se(3) (Mueller)
    Eigen::Matrix4f cross_se3;
    cross_se3 = xi_i_se3 * xi_j_se3 - xi_j_se3 * xi_i_se3;
    return cross_se3;
}

Eigen::Matrix<float, 6, 1>  screws4robotics::lb(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6) {
    // Lie bracket operator on R6, as in [1]/p.175
    // Screw Product of twists \in R6 (Mueller),
    // Generalization of the cross product on R3 to vectors on R6

    Eigen::Vector3f v_i;
    Eigen::Vector3f w_i;
    Eigen::Vector3f v_j;
    Eigen::Vector3f w_j;
    Eigen::Matrix3f w_i_hat;
    Eigen::Matrix3f w_j_hat;
    Eigen::Matrix4f xi_i_se3;
    Eigen::Matrix4f xi_j_se3;
    Eigen::Matrix4f cross_se3;
    Eigen::Matrix<float, 6, 1> LB;

    splitTwist(xi_i_R6, v_i, w_i);
    splitTwist(xi_j_R6, v_j, w_j);

    w_i_hat = skew(w_i);
    w_j_hat = skew(w_j);

    formTwist(xi_i_se3, v_i, w_i_hat);
    formTwist(xi_j_se3, v_j, w_j_hat);

    cross_se3 = xi_i_se3 * xi_j_se3 - xi_j_se3 * xi_i_se3;
    //LB.block<3, 1>(0,0) =  cross_se3.block<3, 1>(0, 3);
    //LB.block<3, 1>(3,0) <<  cross_se3(2,1), cross_se3(0,2), cross_se3(1,0);
    vee(LB, cross_se3);
    return LB;
}

void screws4robotics::spatialCrossProduct(Eigen::Matrix<float, 6, 6> & A, const Eigen::Matrix<float, 6, 1> xi_R6) {
    // Described in [3]/p.243/eq.(116), "spatial cross product"
    // This should be used for derivatives of the Jacobians, based on Mueller papers.
    Eigen::Vector3f v;
    Eigen::Vector3f w;
    Eigen::Matrix3f v_hat;
    Eigen::Matrix3f w_hat;

    splitTwist(xi_R6, v, w);
    v_hat = skew(v);
    w_hat = skew(w);

    A.block<3, 3>(0, 0) = v_hat;
    A.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero(); 
    A.block<3, 3>(3, 0) = w_hat; 
    A.block<3, 3>(3, 3) = v_hat;
}

Eigen::Matrix<float, 6, 1>  screws4robotics::screwProduct(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6) {
    // Lie bracket operator, as in [3]/p.243/eq.(115), "screw product"
    Eigen::Matrix<float, 6, 1> LB;
    Eigen::Matrix<float, 6, 6> ad_twist; // sqp
    spatialCrossProduct(ad_twist, xi_i_R6); // Mueller defined adjoint, or "spatial cross product"
    LB = ad_twist * xi_j_R6;
    return LB;
}

/*
 *   Print utilities, only for Arduino MCU's
 */

void screws4robotics::printVector(Stream& serialPort, const Eigen::Vector3f& v) {
  serialPort.print("Vector: [");
  serialPort.print(v[0]);
  serialPort.print(" ");
  serialPort.print(v[1]);
  serialPort.print(" ");
  serialPort.print(v[2]);
  serialPort.println("]");
}

void screws4robotics::printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> T ) {
    printMatrix(T);
}

void screws4robotics::printTwist(Stream& serialPort, Eigen::Matrix4f T ) {
    printMatrix(T);
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
template void screws4robotics::printMatrix<6,3>(Eigen::Matrix<float, 6, 3>&);
} // namespace screws_math_ns

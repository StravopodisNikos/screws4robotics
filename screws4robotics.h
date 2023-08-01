#ifndef screws4robotics_H
#define screws4robotics_H

#include "Arduino.h"
#include <Eigen30.h>
#include <EigenAVR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

/*
 *  C++ Library(developed to run on Arduino) for implementation
 *  of Screw Theory tools. Special Thanks and Citations:
 *  [1] Murray, R. M., Li, Z., Sastry, S. S., & Sastry, S. S. (1994). A mathematical introduction to robotic manipulation. CRC press.
 *  [2] Müller, A. (2018). Screw and Lie group theory in multibody kinematics: Motion representation and recursive kinematics of tree-topology systems. Multibody System Dynamics, 43(1), 37-70.
 *  [3] Müller, A. (2018). Screw and Lie group theory in multibody dynamics: recursive algorithms and equations of motion of tree-topology systems. Multibody System Dynamics, 42(2), 219-248.
 *  
 *  MATLAB functions for study and comprehension ONLY of the above CAN BE FOUND @screw_dynamics2 repo.
 * 
 *  Arduino test file: test-screws.ino
 */

namespace screws_utils_ns {

    class screws4robotics
    {
    private:
        float _st;
        float _ct;
        
    public:
        screws4robotics();
        ~screws4robotics();

        // main utils
        Eigen::Vector3f crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
        void formTwist(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f v, Eigen::Vector3f w);
        void formTwist(Eigen::Matrix4f & xi_se3, Eigen::Vector3f v, Eigen::Matrix3f wHat);
        void splitTwist(const Eigen::Matrix<float, 6, 1> xi_R6, Eigen::Vector3f & v, Eigen::Vector3f & w);
        void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Vector3f & w);
        void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Matrix3f & wHat);   
        Eigen::Matrix3f skew(const Eigen::Vector3f& w);
        Eigen::Vector3f unskew(const Eigen::Matrix3f& wHat);
        void vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3);
        void wedge(Eigen::Matrix4f& xi , Eigen::Matrix<float, 6, 1> xi_R6);
        Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
        Eigen::Isometry3f twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta);
        Eigen::Isometry3f twistExp(const Eigen::Matrix4f& xi, float theta); 
        void ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
        void Ad(Eigen::Matrix<float, 6, 6> & Ad, const Eigen::Isometry3f& Ci );
        //Eigen::Matrix<float, 6, 6> adBold(Eigen::Matrix<float, 6, 1> xi_i_R6);
        void iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
        Eigen::Matrix4f  lb(Eigen::Matrix4f xi_i_se3, Eigen::Matrix4f xi_j_se3);
        Eigen::Matrix<float, 6, 1> lb(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);
        void spatialCrossProduct(Eigen::Matrix<float, 6, 6> & A, const Eigen::Matrix<float, 6, 1> xi_R6);
        Eigen::Matrix<float, 6, 1> screwProduct(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);

        // prints
        void printVector(Stream& serialPort, const Eigen::Vector3f& v);
        void printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> T );
        void printTwist(Stream& serialPort, Eigen::Matrix4f T );
        void printTransform(Stream& serialPort, const Eigen::Isometry3f& tf);
        template <int Rows, int Cols>
        void printMatrix(Eigen::Matrix<float, Rows, Cols>& matrix);
    };
}
#endif //screws4robotics
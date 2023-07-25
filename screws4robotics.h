#ifndef screws4robotics_H
#define screws4robotics_H

#include "Arduino.h"
#include <Eigen30.h>
#include <EigenAVR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

/*
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
        Eigen::Matrix3f skew(const Eigen::Vector3f& w);
        Eigen::Vector3f unskew(const Eigen::Matrix3f& wHat);
        void vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3);
        void wedge(Eigen::Matrix4f& xi , Eigen::Matrix<float, 6, 1> xi_R6);
        Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
        Eigen::Isometry3f twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta);
        Eigen::Isometry3f twistExp(const Eigen::Matrix4f& xi, float theta); 
        void ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
        void iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );

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
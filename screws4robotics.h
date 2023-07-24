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

namespace screws_math_ns {

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
        void formTwist( Eigen::Matrix<float, 6, 1> & T, const Eigen::Vector3f& v, const Eigen::Vector3f& w);
        Eigen::Matrix3f skew(const Eigen::Vector3f& w);
        Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
        Eigen::Isometry3f twistExp(Eigen::Matrix<float, 6, 1> & xi, float theta);
        void ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
        void iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );

        // prints
        void printVector(Stream& serialPort, const Eigen::Vector3f& v);
        void printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> & T );
        void printTransform(Stream& serialPort, const Eigen::Isometry3f& tf);
        template <int Rows, int Cols>
        void printMatrix(Eigen::Matrix<float, Rows, Cols>& matrix);
    };
}
#endif //screws4robotics
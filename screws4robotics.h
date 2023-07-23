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
        Eigen::Vector3f crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
        void printVector(Stream& serialPort, const Eigen::Vector3f& v);
        void formTwist( Eigen::Matrix<float, 6, 1> & T, const Eigen::Vector3f& v, const Eigen::Vector3f& w);
        void printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> & T );
        Eigen::Matrix3f skew(const Eigen::Vector3f& w);
        Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
        Eigen::Isometry3f twistExp(Eigen::Matrix<float, 6, 1> & xi, float theta);
        void printTransform(Stream& serialPort, const Eigen::Isometry3f& tf);
    };

}
#endif //screws4robotics
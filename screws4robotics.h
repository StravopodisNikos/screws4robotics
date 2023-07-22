#ifndef screws4robotics_H
#define screws4robotics_H

#include "Arduino.h"
#include <Eigen30.h>
#include <EigenAVR.h>

/*
 *  Arduino test file: test-screws.ino
 */

namespace screws_math_ns {

    class screws4robotics
    {
    private:
        /* data */
    public:
        screws4robotics();
        ~screws4robotics();
        Eigen::Vector3f crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
        void printVector(Stream& serialPort, const Eigen::Vector3f& v);
        void formTwist( Eigen::Matrix<float, 6, 1> & T, const Eigen::Vector3f& v, const Eigen::Vector3f& w);
        void printTwist(Stream& serialPort, Eigen::Matrix<float, 6, 1> & T );
    };
    
       
}
#endif //screws4robotics
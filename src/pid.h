#ifndef PID_H_
#define PID_H_

#include <Eigen/Dense>

using namespace Eigen;

// n-dimensional PID filter

class PID{

public:
    PID();

    // Compute output of filter given the state error
    Vector3f compute(Vector3f e, float dt);


    void setGains(Vector3f gP, Vector3f gI, Vector3f gD);

 //   void set

    void setLimit(Vector3f min, Vector3f max);



private:

    Vector3f gainP;
    Vector3f gainI;
    Vector3f gainD;

    Vector3f lastE;
    Vector3f sumE;

};




#endif

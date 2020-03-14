//
// Created by chrisliu on 2020/2/26.
//

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class PayloadController
{
public:

    double massQuadcopter;

    Eigen::Vector3d cAngularVelocity_body;
    Eigen::Matrix3d rotationMatrix_BuW;

    Eigen::Vector3d positionBody;
    Eigen::Vector3d cDesiredPositionBody;
    Eigen::Vector3d pDesiredPositionBody;

    Eigen::Quaterniond desiredQuaterniondBody;

    Eigen::Vector3d velocityBody;
    Eigen::Vector3d desiredVelocityBody;

    Eigen::Vector3d acceleration;

    void initializeParameter(double inputMassQuadcopter);

    void updatePastStates();

    void updateTargetStates(Eigen::Vector3d positionTarget,Eigen::Quaterniond quaternionTarget);

    Eigen::Vector4d getRevs();


private:

    Eigen::Vector3d getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector);

    Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix);


    Eigen::Vector4d getAllocatedRevs(double Force, Eigen::Vector3d Moment);
};


#endif //FLIGHT_CONTROLLER_H

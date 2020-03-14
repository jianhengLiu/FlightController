//
// Created by chrisliu on 2020/2/26.
//

#include "flightController.h"

using namespace std;

void PayloadController::initializeParameter(double inputMassQuadcopter)
{
    massQuadcopter = inputMassQuadcopter;

    cDesiredPositionBody.Zero();
}

void PayloadController::updatePastStates()
{
    pDesiredPositionBody = cDesiredPositionBody;
}

void PayloadController::updateTargetStates(Eigen::Vector3d positionTarget, Eigen::Quaterniond quaternionTarget)
{
    cDesiredPositionBody = positionTarget;
    desiredVelocityBody = getVectorD(cDesiredPositionBody, pDesiredPositionBody);
    desiredQuaterniondBody = quaternionTarget;

}


Eigen::Vector3d PayloadController::getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector)
{
    return cVector - pVector;
}

Eigen::Vector3d PayloadController::antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix)
{
    Eigen::Vector3d vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));

    return vector;
}

Eigen::Vector4d PayloadController::getAllocatedRevs(double Force, Eigen::Vector3d Moment)
{
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, -sqrt2, sqrt2, 1,
            1, -sqrt2, -sqrt2, -1,
            1, sqrt2, -sqrt2, 1,
            1, sqrt2, sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input(Force, Moment.x(), Moment.y(), Moment.z());
    Eigen::Vector4d revs = Minvese * input;
    if (revs.x() < 0)
    {
        revs.x() = 0;
    }
    if (revs.y() < 0)
    {
        revs.y() = 0;
    }
    if (revs.z() < 0)
    {
        revs.z() = 0;
    }
    if (revs.w() < 0)
    {
        revs.w() = 0;
    }
    revs.x() = sqrt(revs.x());
    revs.y() = sqrt(revs.y());
    revs.z() = sqrt(revs.z());
    revs.w() = sqrt(revs.w());
    return revs;
}

Eigen::Vector4d PayloadController::getRevs()
{
    Eigen::Vector3d errorPosition = positionBody - cDesiredPositionBody;
    Eigen::Vector3d errorVelocity = velocityBody - desiredVelocityBody;

    const Eigen::Vector3d k_bx = Eigen::Vector3d(2.0, 2.0, 1.5);
    const Eigen::Vector3d k_bv = Eigen::Vector3d(2.5, 2.5, 2.5);

    Eigen::Vector3d e3(0, 0, -1);

    Eigen::Vector3d F_n = -errorPosition.cwiseProduct(k_bx) - errorVelocity.cwiseProduct(k_bv);

    F_n += (massQuadcopter) * (-e3 * 9.8);

    Eigen::Vector3d F = F_n;

    //保方向饱和函数
    if (F.norm() > 15)
    {
        F = 15 * F / F.norm();
    }

    //2,1,0->ZYX
    Eigen::Vector3d eulerAngle = desiredQuaterniondBody.matrix().eulerAngles(2, 1, 0);
    Eigen::Vector3d b1_des(cos(eulerAngle.x()), sin(eulerAngle.x()), 0);

    Eigen::Vector3d b3_des = F / F.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des /= b2_des.norm();

    Eigen::Matrix3d desiredRotationMatrix;
    desiredRotationMatrix.col(0) = b2_des.cross(b3_des);
    desiredRotationMatrix.col(1) = b2_des;
    desiredRotationMatrix.col(2) = b3_des;

    Eigen::Vector3d errorRotation =
            0.5 * antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * rotationMatrix_BuW -
                                               rotationMatrix_BuW.transpose() * desiredRotationMatrix));

    Eigen::Vector3d errorAngular = cAngularVelocity_body;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

//    Eigen::Matrix3d J_q;
//    J_q << 0.1, 0, 0,
//            0, 0.1, 0,
//            0, 0, 0.1;
    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 1);
    const Eigen::Vector3d k_d = Eigen::Vector3d(0.8, 0.8, 1.5);
    Eigen::Vector3d moment =
            -errorRotation.cwiseProduct(k_p) +
            errorAngular.cwiseProduct(k_d);// + cAngularVelocity_body.cross(J_q * cAngularVelocity_body);

//            保方向饱和函数
    if (moment.norm() > 1)
    {
        moment = 1 * moment / moment.norm();
    }

    double liftForce = b3_des.transpose() * F;

    Eigen::Vector4d revs = getAllocatedRevs(liftForce, moment);

    return revs;
}
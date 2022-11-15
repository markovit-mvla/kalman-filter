#include "DrivetrainPoseEstimator.h"

void DrivetrainPoseEstimator::update(frc::DifferentialDriveWheelSpeeds wheelSpeeds, double leftDist, double rightDist)
{
    estimator_.Update(gyro.GetAngle(), wheelSpeeds, leftDist, rightDist);

    /*
    auto tv = network_table->GetNumber("tv", 0.0);
    if (tv)
    {
        auto res = network_table->GetNumber("camtran", 0.0);
    }
    */
}

void DrivetrainPoseEstimator::resetToPose(frc::Pose2d pose)
{
    estimator_.ResetPosition(pose, gyro.GetAngle());
}

const frc::Pose2d DrivetrainPoseEstimator::getPoseEst()
{
    return estimator_.GetEstimatedPosition();
}
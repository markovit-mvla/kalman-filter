#include "DrivetrainPoseEstimator.h"

#include <frc/Timer.h>

void DrivetrainPoseEstimator::Drive(
    units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot, bool field_relative)
{
    auto states = kinematics_.ToSwerveModuleStates(
        field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, gyro_.GetRotation2d())
        : frc::ChassisSpeeds{xSpeed, ySpeed, rot}  
    );

    kinematics_.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl_, fr_, bl_, br_] = states;
    frontLeft_.SetDesiredState(fl_);
    frontRight_.SetDesiredState(fr_);
    backLeft_.SetDesiredState(bl_);
    backRight_.SetDesiredState(br_);
}

/**
* @todo Continue working on vision measurements using Limelight
*/
void DrivetrainPoseEstimator::UpdateOdometry()
{
    estimator_.Update(gyro_.GetRotation2d(), frontLeft_.getState(),
        frontRight_.getState(), backLeft_.getState(), backRight_.getState());

    estimator_.AddVisionMeasurement(
        limelight_.getPose,
        frc::Timer::GetFPGATimestamp() - 0.3_s;
    );
}

void DrivetrainPoseEstimator::resetToPose(frc::Pose2d pose)
{
    estimator_.ResetPosition(pose, gyro.GetRotation2d());
}

const frc::Pose2d DrivetrainPoseEstimator::getPoseEst()
{
    return estimator_.GetEstimatedPosition();
}
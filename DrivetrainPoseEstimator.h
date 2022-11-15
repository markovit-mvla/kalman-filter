#pragma once

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>

#include "Limelight.h"
#include "SwerveModule.h"
#include "Constants.h"

/**
* Drivetrain for SwerveDrive
*/
class DriveTrainPoseEstimator
{
    public:
        DriveTrainPoseEstimator() { gyro_.Reset(); };

        void UpdateOdometry();

        void Drive(
            units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
            units::radians_per_second_t rot, bool field_relative);

        void resetToPose(frc::Pose2d pose);

        frc::Pose2d getPoseEst();
    private:
        /* Possibly need to adjust numbers */
        frc::Translation2d frontLeftLocation_{+0.381_m, +0.381_m};
        frc::Translation2d frontRightLocation_{+0.381_m, -0.381_m};
        frc::Translation2d backLeftLocation_{-0.381_m, +0.381_m};
        frc::Translation2d backRightLocation_{-0.381_m, -0.381_m};

        SwerveModule frontLeft_{1, 2, 0, 1, 2, 3};
        SwerveModule frontRight_{3, 4, 4, 5, 6, 7};
        SwerveModule backLeft_{5, 6, 8, 9, 10, 11};
        SwerveModule backRight_{7, 8, 12, 13, 14, 15};

        frc::SwerveDriveKinematics<4> kinematics_{
            frontLeftLocation_, frontRightLocation_,
            backLeftLocation_, backRightLocation_
        };

        frc::SwerveDrivePoseEstimator estimator_{
            frc::Rotation2d, frc::Pose2d, kinematics_, 
            /* Need to test and determine hyperparameters */
            {0.1, 0.1, 0.1}, {0.05}, {0.1, 0.1, 0.1}
        };

        frc::AnalogGyro gyro_{0};
        Limelight * limelight_;
}
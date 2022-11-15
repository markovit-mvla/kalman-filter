#include "frc/estimator/SwerveDrivePoseEstimator"
#include "frc/StateSpaceUtil.h"
#include "SwerveDrive.h"
#include <frc/AnalogGyro.h>
#include "Limelight.h"

class DriveTrainPoseEstimator
{
    public:
        DriveTrainPoseEstimator() { network_table = Limelight::GetNetworkTable() };
        void update(frc::DifferentialDriveWheelSpeeds wheelSpeeds, double leftDist, double rightDist);
        void resetToPose(frc::Pose2d pose);
        frc::Pose2d getPoseEst();
    private:
        frc::SwerveDrivePoseEstimator estimator_{
            frc::Rotation2d, frc::Pose2d,
            frc::MakeMatrix<5, 1>(0.01, 0.01, 0.01, 0.01, 0.01),
            frc::MakeMatrix<3, 1>(0.01, 0.01, 0.01),
            frc::MakeMatrix<3, 1>(0.01, 0.01, 0.01)
        };
        frc::AnalogGyro gyro{0};
        std::shared_ptr<nt::NetworkTable> network_table;
}


package frc.lib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.drive.DriveBase;
import frc.robot.drive.GyroIO;

public class PoseEstimator {

  public GyroIO m_GyroIO;

  public DriveBase m_SwerveModulePoss;

  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
       Drivetrain.kKinematics,
       m_GyroIO.getHeading(),
       m_SwerveModulePoss.getModulePositions(),
       new Pose2d());
 
  public final Pose2d getPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }
}

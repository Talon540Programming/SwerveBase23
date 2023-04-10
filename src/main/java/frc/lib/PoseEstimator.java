package frc.lib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator extends SwerveDrivePoseEstimator {
  private static PoseEstimator instance;

  public static void createInstance(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroHeading,
      SwerveModulePosition... modulePositions) {
    instance = new PoseEstimator(kinematics, gyroHeading, modulePositions);
  }

  public static PoseEstimator getInstance() {
    if (instance == null) {
      throw new IllegalStateException(
          "The PoseEstimator hasn't been created yet. Create it first using PoseEstimator::createInstance");
    } else {
      return instance;
    }
  }

  private PoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroHeading,
      SwerveModulePosition... modulePositions) {
    super(kinematics, gyroHeading, modulePositions, new Pose2d());
  }
}

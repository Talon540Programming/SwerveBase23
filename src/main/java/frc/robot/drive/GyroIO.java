package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO extends LoggedIO<GyroIO.GyroInputs> {
  @AutoLog
  public class GyroInputs {
    public boolean Connected;

    public double GyroPositionYawRad;
    public double GyroPositionPitchRad;
    public double GyroPositionRollRad;

    public double GyroRateYawRadPerSecond;
    public double GyroRatePitchRadPerSecond;
    public double GyroRateRollRadPerSecond;

    public double AccelXGForce;
    public double AccelYGForce;
    public double AccelZGForce;
  }

  /**
   * Get the heading of the robot.
   *
   * @return robot heading.
   */
  default Rotation2d getHeading() {
    return new Rotation2d();
  }

  /** Reset the yaw of the gyro to 0. */
  default void resetHeading() {}
}

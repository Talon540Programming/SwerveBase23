package frc.robot.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO extends LoggedIO<GyroIO.GyroInputs> {
  @AutoLog
  public class GyroInputs {
    public boolean Connected;

    public double RollPositionRad;
    public double PitchPositionRad;
    public double YawPositionRad;

    public double RollRateRadPerSecond;
    public double PitchRateRadPerSecond;
    public double YawRateRadPerSecond;

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

  default void incrementHeading(double incrementRad) {}
}

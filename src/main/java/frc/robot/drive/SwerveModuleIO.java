package frc.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleIO.SwerveModuleInputs> {
  @AutoLog
  public class SwerveModuleInputs {
    public double DriveCurrentAmps;
    public double DriveTempCelsius;
    public double DistanceTraveledMeters;
    public double VelocityMetersPerSecond;

    public double TurnCurrentAmps;
    public double TurnTempCelsius;
    public double TurnPositionRad;
    public double TurnVelocityRadPerSec;

    public double AbsoluteModulePositionRad;
  }

  default SwerveModuleState getState() {
    return new SwerveModuleState();
  }

  default SwerveModulePosition getPosition() {
    return new SwerveModulePosition();
  }

  default void setState(SwerveModuleState state) {}

  default void resyncEncoders() {}

  default void stop() {}

  default void setNeutralMode(
      Constants.NeutralMode driveNeutralMode, Constants.NeutralMode steerNeutralMode) {}
}

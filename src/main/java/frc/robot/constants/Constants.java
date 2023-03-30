package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  /**
   * Whether advanced logging should be enabled. This can be disabled if there is too much going on.
   */
  public static final boolean kAdvancedLoggingEnabled = true;

  private static RobotType kRobotType = RobotType.ROBOT_SIMBOT;
  public static final double loopPeriodSecs = 0.02;

  public enum RobotMode {
    REAL,
    REPLAY,
    SIM
  }

  public enum RobotType {
    ROBOT_SWERVE,
    ROBOT_SIMBOT
  }

  public static RobotType getRobotType() {
    if (RobotBase.isReal() && kRobotType == RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to SIM but it isn't a SIM, setting it to Competition Robot as redundancy.",
          false);
      kRobotType = RobotType.ROBOT_SWERVE;
    }

    if (RobotBase.isSimulation() && kRobotType != RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to REAL but it is a SIM, setting it to SIMBOT as redundancy.", false);
      kRobotType = RobotType.ROBOT_SIMBOT;
    }

    return kRobotType;
  }

  public static RobotMode getRobotMode() {
    return switch (getRobotType()) {
      case ROBOT_SWERVE -> RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
      case ROBOT_SIMBOT -> RobotMode.SIM;
    };
  }

  public static class Drivetrain {
    // MODULE LOAD ORDER IS FRONT LEFT -> FRONT RIGHT -> BACK LEFT -> BACK RIGHT

    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(
            FrontLeft.kCenterOffset, // FRONT LEFT
            FrontRight.kCenterOffset, // FRONT RIGHT
            BackLeft.kCenterOffset, // BACK LEFT
            BackRight.kCenterOffset // BACK RIGHT
            );

    public static final boolean kSteerMotorInverted = true;

    public static final double kMaxVelocityMetersPerSecond = Units.feetToMeters(12.0); // SDS MK4 L1
    // public static final double kMaxVelocityMetersPerSecond = Units.feetToMeters(14.5); // SDS MK4
    // L2
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxRotationVelocityRadPerSecond = 2 * Math.PI;

    public static final double kDriveGearRatio = 8.14; // SDS MK4 L1
    // public static final double kDriveGearRatio = 6.75; // SDS MK4 L2
    public static final double kSteerGearRatio = 12.8;

    public static final double kWheelRadiusInches = 2;
    public static final double KWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

    public static final double kDriveMotorConversionFactor =
        2 * Math.PI * KWheelRadiusMeters / kDriveGearRatio;
    public static final double kSteerMotorConversionFactor = 2 * Math.PI / kSteerGearRatio;

    public static class FrontLeft {
      public static final Translation2d kCenterOffset =
          new Translation2d(Units.inchesToMeters(8.825180), Units.inchesToMeters(8.825180));
      public static final double kMagneticOffsetDegrees = 0.0; // TODO
    }

    public static class FrontRight {
      public static final Translation2d kCenterOffset =
          new Translation2d(Units.inchesToMeters(8.825180), -Units.inchesToMeters(8.825180));
      public static final double kMagneticOffsetDegrees = 0.0; // TODO
    }

    public static class BackLeft {
      public static final Translation2d kCenterOffset =
          new Translation2d(-Units.inchesToMeters(8.825180), Units.inchesToMeters(8.825180));
      public static final double kMagneticOffsetDegrees = 0.0; // TODO
    }

    public static class BackRight {
      public static final Translation2d kCenterOffset =
          new Translation2d(-Units.inchesToMeters(8.825180), -Units.inchesToMeters(8.825180));
      public static final double kMagneticOffsetDegrees = 0.0; // TODO
    }

    public static class ControlValues {
      public static class Drive {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kFF = 0; // TODO
      }

      public static class Steer {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }

  public enum NeutralMode {
    BRAKE,
    COAST;

    /**
     * Convert the Neutral mode to one used by the SparkMax API.
     *
     * @return SparkMax Idle Mode.
     */
    public CANSparkMax.IdleMode toIdleMode() {
      return switch (this) {
        case BRAKE -> CANSparkMax.IdleMode.kBrake;
        case COAST -> CANSparkMax.IdleMode.kCoast;
      };
    }
  }
}

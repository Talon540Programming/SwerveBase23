package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  /**
   * Whether advanced logging should be enabled. This can be disabled if there is too much going on.
   */
  public static final boolean kAdvancedLoggingEnabled = true;

  private static RobotType kRobotType = RobotType.ROBOT_SWERVE;
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

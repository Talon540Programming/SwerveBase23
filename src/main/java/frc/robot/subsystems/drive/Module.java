package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Drivetrain.DriveCoefficients;
import frc.robot.constants.Constants.Drivetrain.TurnCoefficients;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int kModuleIndex;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final PIDController m_driveController;
  private final PIDController m_turnController;

  private Rotation2d m_turnAngleSetpoint = null;
  private Double m_driveVelocitySetpoint = null;

  private Rotation2d m_turnRelativeOffset = null;
  private double m_lastPositionMeters;
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  public Module(int index, ModuleIO io) {
    this.kModuleIndex = index;
    this.m_io = io;

    switch (Constants.getRobotMode()) {
      case REPLAY, REAL:
        m_driveFeedforward =
            new SimpleMotorFeedforward(
                DriveCoefficients.kS, DriveCoefficients.kV, DriveCoefficients.kA);
        m_driveController =
            new PIDController(DriveCoefficients.kP, DriveCoefficients.kI, DriveCoefficients.kD);
        m_turnController =
            new PIDController(TurnCoefficients.kP, TurnCoefficients.kI, TurnCoefficients.kD);
        break;
      case SIM:
        m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        m_driveController = new PIDController(0.1, 0, 0);
        m_turnController = new PIDController(10.0, 0, 0);
        break;
      default:
        m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
        m_driveController = new PIDController(0, 0, 0);
        m_turnController = new PIDController(0, 0, 0);
        break;
    }

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  /** Updates the inputs of the module. Should be called before {@link #periodic} */
  public void updateInputs() {
    m_io.updateInputs(m_inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + kModuleIndex, m_inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (m_turnRelativeOffset == null && m_inputs.turnAbsolutePosition.getRadians() != 0.0) {
      m_turnRelativeOffset = m_inputs.turnAbsolutePosition.minus(m_inputs.turnPosition);
    }

    // Run closed loop turn control
    if (m_turnAngleSetpoint != null) {
      m_io.setTurnVoltage(
          m_turnController.calculate(getAngle().getRadians(), m_turnAngleSetpoint.getRadians()));

      // Run closed loop drive control. Only if turn closed loop is enabled.
      if (m_driveVelocitySetpoint != null) {
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint =
            m_driveVelocitySetpoint * Math.cos(m_turnController.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / Constants.Drivetrain.kWheelRadiusMeters;
        m_io.setDriveVoltage(
            m_driveFeedforward.calculate(velocityRadPerSec)
                + m_driveController.calculate(m_inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    // Calculate position deltas for odometry
    int deltaCount =
        Math.min(m_inputs.odometryDrivePositionsRad.length, m_inputs.odometryTurnPositions.length);
    positionDeltas = new SwerveModulePosition[deltaCount];
    for (int i = 0; i < deltaCount; i++) {
      double positionMeters =
          m_inputs.odometryDrivePositionsRad[i] * Constants.Drivetrain.kWheelRadiusMeters;
      Rotation2d angle =
          m_inputs.odometryTurnPositions[i].plus(
              m_turnRelativeOffset != null ? m_turnRelativeOffset : new Rotation2d());
      positionDeltas[i] = new SwerveModulePosition(positionMeters - m_lastPositionMeters, angle);
      m_lastPositionMeters = positionMeters;
    }
  }

  /**
   * Runs the module with the specified setpoint state. Must be called periodically. Returns the
   * optimized state.
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    m_turnAngleSetpoint = optimizedState.angle;
    m_driveVelocitySetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /**
   * Set the module output to the specified voltages
   *
   * @param driveVoltage drive voltage [-12, 12]
   * @param turnVoltage turn voltage [-12, 12]
   */
  public void runVoltage(double driveVoltage, double turnVoltage) {
    m_io.setDriveVoltage(MathUtil.clamp(driveVoltage, -12, 12));
    m_io.setTurnVoltage(MathUtil.clamp(turnVoltage, -12, 12));

    m_turnAngleSetpoint = null;
    m_driveVelocitySetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void disable() {
    m_io.setTurnVoltage(0.0);
    m_io.setDriveVoltage(0.0);

    m_turnAngleSetpoint = null;
    m_driveVelocitySetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
    m_io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (m_turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return m_inputs.turnPosition.plus(m_turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * Constants.Drivetrain.kWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * Constants.Drivetrain.kWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }
}

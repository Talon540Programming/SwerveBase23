package frc.robot.drive;
//
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import frc.robot.constants.Constants;
//
// public class SwerveModuleIOSim implements SwerveModuleIO {
//     // private final FlywheelSim m_driveGearbox = new FlywheelSim(DCMotor.getNEO(1),
// Constants.Drivetrain.kDriveGearRatio, 0.025);
//     // private final FlywheelSim m_steerGearbox = new FlywheelSim(DCMotor.getNEO(1),
// Constants.Drivetrain.kSteerGearRatio, 0.004);
//
//     private final DCMotorSim m_driveGearbox = new DCMotorSim(DCMotor.getNEO(1),
// Constants.Drivetrain.kDriveGearRatio, 0.025);
//     private final DCMotorSim m_steerGearbox = new DCMotorSim(DCMotor.getNEO(1),
// Constants.Drivetrain.kSteerGearRatio, 0.004);
//
//     private double turnRelativePositionRad = 0.0;
//     private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
//     private double driveAppliedVolts = 0.0;
//     private double turnAppliedVolts = 0.0;
//
//     private final PIDController m_driveController = new PIDController(
//             Constants.Drivetrain.ControlValues.Drive.kP,
//             Constants.Drivetrain.ControlValues.Drive.kI,
//             Constants.Drivetrain.ControlValues.Drive.kD
//     );
//     private final PIDController m_steerController = new PIDController(
//             Constants.Drivetrain.ControlValues.Steer.kP,
//             Constants.Drivetrain.ControlValues.Steer.kI,
//             Constants.Drivetrain.ControlValues.Steer.kD
//     );
//
//     public SwerveModuleIOSim() {
//         m_steerController.enableContinuousInput(-Math.PI, Math.PI);
//     }
//
//     @Override
//     public void updateInputs(SwerveModuleInputs inputs) {
//         m_driveGearbox.update(Constants.loopPeriodSecs);
//         m_steerGearbox.update(Constants.loopPeriodSecs);
//
//         inputs.DriveCurrentAmps = m_driveGearbox.getCurrentDrawAmps();
//
//         inputs.TurnPositionRad = m_steerGearbox.getAngularPositionRad();
//         inputs.TurnVelocityRadPerSec = m_steerGearbox.getAngularVelocityRadPerSec();
//
//         // Units.radiansToRotations(0) * Constants.Drivetrain.kDriveMotorConversionFactor;
//
//
//
//     }
//
//     @Override
//     public SwerveModulePosition getPosition() {
//         return SwerveModuleIO.super.getPosition();
//     }
//
//     @Override
//     public SwerveModuleState getState() {
//         return SwerveModuleIO.super.getState();
//     }
//
//     @Override
//     public void setState(SwerveModuleState state) {
//         // Optimize the SwerveModuleState
//         state = SwerveModuleState.optimize(state,
// Rotation2d.fromRadians(turnAbsolutePositionRad));
//
//         double driveOutput = m_driveController.calculate();
//     }
// }

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private final SwerveModuleState m_currentState = new SwerveModuleState();
  private SwerveModuleState m_desiredState = new SwerveModuleState();
  private final SwerveModulePosition m_currentPosition = new SwerveModulePosition();

  @Override
  public SwerveModuleState getState() {
    return m_currentState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return m_currentPosition;
  }

  @Override
  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, m_currentState.angle);

    m_desiredState = state;
  }

  @Override
  public void stop() {
    m_currentState.speedMetersPerSecond = 0;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    double oldAngleRad = m_currentPosition.angle.getRadians();
    double updatedAngle =
        MathUtil.interpolate(
            m_currentPosition.angle.getRadians(), m_desiredState.angle.getRadians(), 1);

    // Update State
    m_currentState.angle = Rotation2d.fromRadians(updatedAngle);
    m_currentState.speedMetersPerSecond = m_desiredState.speedMetersPerSecond;

    // Update Pose
    m_currentPosition.distanceMeters +=
        m_currentState.speedMetersPerSecond * Constants.loopPeriodSecs;
    m_currentPosition.angle = m_currentState.angle;

    inputs.DistanceTraveledMeters = m_currentPosition.distanceMeters;
    inputs.VelocityMetersPerSecond = m_currentState.speedMetersPerSecond;
    inputs.TurnVelocityRadPerSec =
        (m_currentPosition.angle.getRotations() - oldAngleRad) / Constants.loopPeriodSecs;
    inputs.TurnPositionRad = m_currentPosition.angle.getRadians();
  }
}

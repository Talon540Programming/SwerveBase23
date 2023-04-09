package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class SwerveModuleIOSim2 implements SwerveModuleIO {
  private final DCMotorSim m_driveGearbox =
      new DCMotorSim(DCMotor.getNEO(1), Constants.Drivetrain.kDriveGearRatio, 0.025);
  private final DCMotorSim m_steerGearbox =
      new DCMotorSim(DCMotor.getNEO(1), Constants.Drivetrain.kSteerGearRatio, 0.004);

  private double driveAppliedVoltage;
  private double steerAppliedVoltage;

  private final PIDController m_driveController =
      new PIDController(
          Constants.Drivetrain.ControlValues.Drive.kP,
          Constants.Drivetrain.ControlValues.Drive.kI,
          Constants.Drivetrain.ControlValues.Drive.kD);

  private final PIDController m_steerController =
      new PIDController(
          Constants.Drivetrain.ControlValues.Steer.kP,
          Constants.Drivetrain.ControlValues.Steer.kI,
          Constants.Drivetrain.ControlValues.Steer.kD);

  public SwerveModuleIOSim2() {
    m_steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    m_driveGearbox.update(Constants.loopPeriodSecs);
    m_steerGearbox.update(Constants.loopPeriodSecs);

    inputs.DriveCurrentAmps = m_driveGearbox.getCurrentDrawAmps();
    inputs.DriveAppliedVoltage = driveAppliedVoltage;
    inputs.SteerCurrentAmps = m_steerGearbox.getCurrentDrawAmps();
    inputs.SteerAppliedVoltage = steerAppliedVoltage;

    inputs.VelocityRadPerSec =
        m_steerGearbox.getAngularVelocityRPM()
            * Constants.Drivetrain.kSteerMotorConversionFactor
            / 60.0;
    inputs.VelocityMetersPerSec =
        m_driveGearbox.getAngularVelocityRPM()
            * Constants.Drivetrain.kDriveMotorConversionFactor
            / 60.0;

    inputs.RelativePositionRad =
        m_steerGearbox.getAngularPositionRotations()
            * Constants.Drivetrain.kSteerMotorConversionFactor;
    inputs.DistanceTraveledMeters =
        m_driveGearbox.getAngularPositionRotations()
            * Constants.Drivetrain.kDriveMotorConversionFactor;
  }

  @Override
  public SwerveModuleState getState() {
    return SwerveModuleIO.super.getState();
  }

  @Override
  public SwerveModulePosition getPosition() {
    return SwerveModuleIO.super.getPosition();
  }

  @Override
  public void setState(SwerveModuleState state) {
    SwerveModuleIO.super.setState(state);
  }

  @Override
  public void resyncEncoders() {
    m_driveGearbox.getOutput().set(0, 0, 0.0);
  }

  @Override
  public void stop() {
    driveAppliedVoltage = 0;
    steerAppliedVoltage = 0;

    m_driveGearbox.setInputVoltage(0);
    m_steerGearbox.setInputVoltage(0);
  }
}

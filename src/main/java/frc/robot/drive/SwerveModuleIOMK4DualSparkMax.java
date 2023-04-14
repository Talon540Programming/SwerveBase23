package frc.robot.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.SparkMaxBurnManager;
import frc.lib.SparkMaxPeriodicFrameConfig;
import frc.robot.constants.Constants;

public class SwerveModuleIOMK4DualSparkMax implements SwerveModuleIO {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_steerMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_steerEncoder;

  private final SparkMaxPIDController m_driveController;
  private final SparkMaxPIDController m_steerController;

  private final WPI_CANCoder m_absoluteEncoder;

  public SwerveModuleIOMK4DualSparkMax(
      int driveMotorId,
      int steerMotorId,
      int absoluteEncoderId,
      boolean steerInverted,
      double driveConversionFactor,
      double steerConversionFactor,
      double absoluteEncoderOffsetDegrees) {
    m_driveMotor = new CANSparkMax(driveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_driveMotor.restoreFactoryDefaults();
      m_steerMotor.restoreFactoryDefaults();
    }

    m_driveMotor.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);
    m_steerMotor.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_steerEncoder = m_steerMotor.getEncoder();

    m_driveController = m_driveMotor.getPIDController();
    m_steerController = m_steerMotor.getPIDController();

    SparkMaxPeriodicFrameConfig.configureIsolated(m_driveMotor);
    SparkMaxPeriodicFrameConfig.configureIsolated(m_steerMotor);

    m_steerMotor.setInverted(steerInverted);

    m_driveMotor.setSmartCurrentLimit(40);
    m_driveMotor.enableVoltageCompensation(12.0);

    m_steerMotor.setSmartCurrentLimit(20);
    m_steerMotor.enableVoltageCompensation(12.0);

    m_driveEncoder.setPositionConversionFactor(driveConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(driveConversionFactor / 60.0);
    m_driveEncoder.setPosition(0.0);
    m_driveEncoder.setMeasurementPeriod(10);
    m_driveEncoder.setAverageDepth(2);

    m_steerEncoder.setPositionConversionFactor(steerConversionFactor);
    m_steerEncoder.setVelocityConversionFactor(steerConversionFactor / 60.0);
    m_steerEncoder.setPosition(0.0);
    m_steerEncoder.setMeasurementPeriod(10);
    m_steerEncoder.setAverageDepth(2);

    m_driveController.setP(Constants.Drivetrain.ControlValues.Drive.kP);
    m_driveController.setI(Constants.Drivetrain.ControlValues.Drive.kI);
    m_driveController.setD(Constants.Drivetrain.ControlValues.Drive.kD);
    m_driveController.setFF(Constants.Drivetrain.ControlValues.Drive.kFF);

    m_steerController.setP(Constants.Drivetrain.ControlValues.Steer.kP);
    m_steerController.setI(Constants.Drivetrain.ControlValues.Steer.kI);
    m_steerController.setD(Constants.Drivetrain.ControlValues.Steer.kD);

    m_steerController.setPositionPIDWrappingEnabled(true);
    m_steerController.setPositionPIDWrappingMinInput(-Math.PI);
    m_steerController.setPositionPIDWrappingMaxInput(Math.PI);

    m_driveMotor.setCANTimeout(0);
    m_steerMotor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_driveMotor.burnFlash();
      m_steerMotor.burnFlash();
    }

    m_absoluteEncoder = new WPI_CANCoder(absoluteEncoderId);

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.magnetOffsetDegrees = absoluteEncoderOffsetDegrees;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

    m_absoluteEncoder.configAllSettings(config);

    setNeutralMode(Constants.NeutralMode.COAST, Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.DriveCurrentAmps = m_driveMotor.getOutputCurrent();
    inputs.DriveTempCelsius = m_driveMotor.getMotorTemperature();
    inputs.DriveAppliedVoltage = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
    inputs.SteerCurrentAmps = m_steerMotor.getOutputCurrent();
    inputs.SteerTempCelsius = m_steerMotor.getMotorTemperature();
    inputs.SteerAppliedVoltage = m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage();

    inputs.VelocityRadPerSec = m_steerEncoder.getVelocity();
    inputs.VelocityMetersPerSec = m_driveEncoder.getVelocity();
    inputs.AbsolutePositionRad = m_absoluteEncoder.getAbsolutePosition();
    inputs.RelativePositionRad = m_steerEncoder.getPosition();
    inputs.DistanceTraveledMeters = m_driveEncoder.getPosition();
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(),
        Rotation2d.fromRadians(m_absoluteEncoder.getAbsolutePosition()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        Rotation2d.fromRadians(m_absoluteEncoder.getAbsolutePosition()));
  }

  @Override
  public void setState(SwerveModuleState state) {
    // Optimize the SwerveModuleState
    state =
        SwerveModuleState.optimize(
            state, Rotation2d.fromRadians(m_absoluteEncoder.getAbsolutePosition()));

    m_driveController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_steerController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void resyncEncoders() {
    m_driveEncoder.setPosition(0);
    m_steerEncoder.setPosition(m_absoluteEncoder.getAbsolutePosition());
  }

  @Override
  public void stop() {
    m_driveMotor.stopMotor();
  }

  @Override
  public void setNeutralMode(
      Constants.NeutralMode driveNeutralMode, Constants.NeutralMode steerNeutralMode) {
    m_driveMotor.setIdleMode(driveNeutralMode.toIdleMode());
    m_steerMotor.setIdleMode(steerNeutralMode.toIdleMode());
  }
}

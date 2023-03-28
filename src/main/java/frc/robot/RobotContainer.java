package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.SparkMaxBurnManager;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.HardwareDevices.SwerveBase.Drivetrain.*;
import frc.robot.drive.*;

public class RobotContainer {
  private DriveBase m_driveBase;

  public RobotContainer() {
    SparkMaxBurnManager.checkBuildStatus();

    if (Constants.getRobotMode() != Constants.RobotMode.REPLAY) {
      switch (Constants.getRobotType()) {
        case ROBOT_SWERVE -> {
          m_driveBase =
              new DriveBase(
                  // FRONT LEFT
                  new SwerveModuleIOMK4DualSparkMax(
                      FrontLeft.kDriveMotorId,
                      FrontLeft.kSteerMotorId,
                      FrontLeft.kAbsoluteEncoderId,
                      Constants.Drivetrain.kTurnMotorInverted,
                      Constants.Drivetrain.kDriveMotorConversionFactor,
                      Constants.Drivetrain.kSteerMotorConversionFactor,
                      Constants.Drivetrain.FrontLeft.kMagneticOffsetDegrees),
                  // FRONT RIGHT
                  new SwerveModuleIOMK4DualSparkMax(
                      FrontRight.kDriveMotorId,
                      FrontRight.kSteerMotorId,
                      FrontRight.kAbsoluteEncoderId,
                      Constants.Drivetrain.kTurnMotorInverted,
                      Constants.Drivetrain.kDriveMotorConversionFactor,
                      Constants.Drivetrain.kSteerMotorConversionFactor,
                      Constants.Drivetrain.FrontRight.kMagneticOffsetDegrees),
                  // BACK LEFT
                  new SwerveModuleIOMK4DualSparkMax(
                      BackLeft.kDriveMotorId,
                      BackLeft.kSteerMotorId,
                      BackLeft.kAbsoluteEncoderId,
                      Constants.Drivetrain.kTurnMotorInverted,
                      Constants.Drivetrain.kDriveMotorConversionFactor,
                      Constants.Drivetrain.kSteerMotorConversionFactor,
                      Constants.Drivetrain.BackLeft.kMagneticOffsetDegrees),
                  // BACK RIGHT
                  new SwerveModuleIOMK4DualSparkMax(
                      BackRight.kDriveMotorId,
                      BackRight.kSteerMotorId,
                      BackRight.kAbsoluteEncoderId,
                      Constants.Drivetrain.kTurnMotorInverted,
                      Constants.Drivetrain.kDriveMotorConversionFactor,
                      Constants.Drivetrain.kSteerMotorConversionFactor,
                      Constants.Drivetrain.BackRight.kMagneticOffsetDegrees),
                  new GyroIOPigeon2(HardwareDevices.SwerveBase.kGyroId));
        }
        case ROBOT_SIMBOT -> {}
      }
    }

    m_driveBase =
        m_driveBase != null
            ? m_driveBase
            : new DriveBase(
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new GyroIO() {});

    configureBindings();
    configureAuto();
  }

  private void configureBindings() {}

  private void configureAuto() {}

  public Command getAutonomousCommand() {
    return null;
  }
}

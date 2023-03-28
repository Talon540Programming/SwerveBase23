package frc.robot.drive.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.drive.DriveBase;
import frc.robot.oi.DriverInterface;

public class DriveControl extends CommandBase {
  private final DriveBase m_driveBase;
  private final DriverInterface m_driverInterface;

  public DriveControl(DriveBase driveBase, DriverInterface driverInterface) {
    m_driveBase = driveBase;
    m_driverInterface = driverInterface;
  }

  @Override
  public void execute() {
    m_driveBase.setFromForces(
        m_driverInterface.getTranslationalLateralPercent()
            * Constants.Drivetrain.kMaxVelocityMetersPerSecond,
        m_driverInterface.getTranslationalMedialPercent()
            * Constants.Drivetrain.kMaxVelocityMetersPerSecond,
        m_driverInterface.getRotationalPercent()
            * Constants.Drivetrain.kMaxRotationVelocityRadPerSecond,
        m_driverInterface.isRobotRelative().getAsBoolean()
            ? DriveBase.DriveMode.kRobot
            : DriveBase.DriveMode.kField);
  }
}

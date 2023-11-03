package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.drive.DriveCommandFactory;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareIds;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.DriveBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final DriveBase m_drive;

  // Controller
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.getRobotMode()) {
      case REAL -> {
        m_drive =
            new DriveBase(
                new GyroIOPigeon2(HardwareIds.kPigeonId),
                new ModuleIOSparkMax(
                    HardwareIds.kFrontLeftDriveId,
                    HardwareIds.kFrontLeftTurnId,
                    HardwareIds.kFrontLeftEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kFrontRightDriveId,
                    HardwareIds.kFrontRightTurnId,
                    HardwareIds.kFrontRightEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kBackLeftDriveId,
                    HardwareIds.kBackLeftTurnId,
                    HardwareIds.kBackLeftEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kBackRightDriveId,
                    HardwareIds.kBackRightTurnId,
                    HardwareIds.kBackRightEncoderId));
      }
      case SIM -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
      }
      default -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    // // Set up FF characterization routines
    // autoChooser.addOption(
    //     "DriveBase FF Characterization",
    //     new FeedForwardCharacterization(
    //         m_drive, m_drive::runCharacterizationVolts, m_drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommandFactory.joystickDrive(
            m_drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            0.1));
    controller.cross().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

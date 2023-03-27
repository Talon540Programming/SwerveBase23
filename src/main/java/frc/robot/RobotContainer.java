package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class RobotContainer {
  public RobotContainer() {
    if (Constants.getRobotMode() != Constants.RobotMode.REPLAY) {
      switch (Constants.getRobotType()) {
        case ROBOT_SWERVE -> {}
        case ROBOT_SIMBOT -> {}
      }
    }

    configureBindings();
    configureAuto();
  }

  private void configureBindings() {}

  private void configureAuto() {}

  public Command getAutonomousCommand() {
    return null;
  }
}

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.generated.BuildConstants;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class LoggerUtil {
  /** Initialize the Logger with the auto-generated data from the build. */
  public static void initializeLoggerMetadata() {
    // Record metadata from generated state file.
    Logger.recordMetadata("ROBOT_NAME", Constants.getRobotType().toString());
    Logger.recordMetadata("RUNTIME_ENVIRONMENT", RobotBase.getRuntimeType().toString());
    Logger.recordMetadata("PROJECT_NAME", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BUILD_DATE", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GIT_DATE", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GIT_BRANCH", BuildConstants.GIT_BRANCH);

    // Set the current GIT state of the robot (helps manage the logs that are saved).
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GIT_STATUS", "All changes committed");
      case 1 -> Logger.recordMetadata("GIT_STATUS", "Uncommitted changes");
      default -> Logger.recordMetadata("GIT_STATUS", "Unknown");
    }
  }
}

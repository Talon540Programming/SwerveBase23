package frc.lib;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.generated.BuildConstants;
import frc.robot.constants.Constants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

/** Utilities used by the AdvantageKit Logger. */
@SuppressWarnings("DataFlowIssue")
public class LoggerUtil {
  /**
   * Initialize the Logger with the auto-generated data from the build.
   *
   * @param logger logger to update.
   */
  public static void initializeLoggerMetadata(Logger logger) {
    // Record metadata from generated state file.
    logger.recordMetadata("ROBOT_NAME", Constants.getRobotType().toString());
    logger.recordMetadata("RUNTIME_ENVIRONMENT", RobotBase.getRuntimeType().toString());
    logger.recordMetadata("PROJECT_NAME", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BUILD_DATE", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GIT_DATE", BuildConstants.GIT_DATE);
    logger.recordMetadata("GIT_BRANCH", BuildConstants.GIT_BRANCH);

    // Set the current GIT state of the robot (helps manage the logs that are saved).
    switch (BuildConstants.DIRTY) {
      case 0 -> logger.recordMetadata("GIT_STATUS", "All changes committed");
      case 1 -> logger.recordMetadata("GIT_STATUS", "Uncommitted changes");
      default -> logger.recordMetadata("GIT_STATUS", "Unknown");
    }
  }

  /**
   * Get the path of the USB drive if it is plugged into the roboRIO. If one isn't found, it will
   * return null.
   *
   * @return path of the USB drive.
   */
  public static String getUSBPath() {
    // Return the path of the USB drive it is plugged in, else, return null.
    try {
      Path drivePath = Path.of("/u").toRealPath();
      return drivePath.toString();
    } catch (IOException e) {
      return null;
    }
  }

  /**
   * Log all the currently connected NetworkTables clients. This method should be called
   * periodically.
   */
  public static void logNTClients() {
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();

    for (ConnectionInfo client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }

    Logger.getInstance().recordOutput("NTClients/Names", clientNames.toArray(new String[0]));
    Logger.getInstance()
        .recordOutput("NTClients/Addresses", clientAddresses.toArray(new String[0]));
  }

  /** Log Commands scheduled by the CommandScheduler to telemetry automatically. */
  public static void initCommandLogging() {
    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.getInstance()
              .recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
  }
}

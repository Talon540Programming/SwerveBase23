package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LoggerUtil;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    LoggerUtil.initializeLoggerMetadata(logger);

    // Set up data receivers & replay source
    switch (Constants.getRobotMode()) {
      case REAL -> {
        String path = LoggerUtil.getUSBPath();

        if (path != null) {
          logger.addDataReceiver(new WPILOGWriter(path));
        } else {
          DriverStation.reportWarning(
              "Unable to locate a USB drive plugged into the roboRIO. Hardware storage logging will be disabled.",
              false);
        }

        logger.addDataReceiver(new NT4Publisher());

        LoggedPowerDistribution.getInstance();
        LoggedDriverStation.getInstance();
        LoggedSystemStats.getInstance();
      }
      case SIM -> logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false);
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
      }
    }

    // Start AdvantageKit logger
    logger.start();

    LoggerUtil.initCommandLogging();

    if (Constants.getRobotMode() == Constants.RobotMode.SIM)
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (Constants.kAdvancedLoggingEnabled) {
      LoggerUtil.logNTClients();
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}

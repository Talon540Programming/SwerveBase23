package frc.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Configures the rates at which different status frames are sent from SparkMaxes. See <a
 * href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">here</a>
 * for different status frames.
 */
public class SparkMaxPeriodicFrameConfig {
  public static void configureLeader(CANSparkMax motorController) {
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configureFollower(CANSparkMax motorController) {
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configureIsolated(CANSparkMax motorController) {
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}

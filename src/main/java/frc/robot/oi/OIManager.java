package frc.robot.oi;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.HardwareDevices;

public class OIManager {
  private final DriverInterface m_driverInterface;

  public OIManager() {
    if (RobotBase.isSimulation()) {
      m_driverInterface = new PS4Driver(HardwareDevices.kDriverControllerPort);
    } else {
      m_driverInterface = new XboxDriver(HardwareDevices.kDriverControllerPort);
    }
  }

  public DriverInterface getDriverInterface() {
    return m_driverInterface;
  }
}

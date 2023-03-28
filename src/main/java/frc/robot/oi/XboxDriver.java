package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDriver implements DriverInterface {
  private final CommandXboxController m_controller;

  public XboxDriver(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double getTranslationalMedialPercent() {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.15);
  }

  @Override
  public double getTranslationalLateralPercent() {
    return MathUtil.applyDeadband(m_controller.getLeftX(), 0.15);
  }

  @Override
  public double getRotationalPercent() {
    return MathUtil.applyDeadband(m_controller.getRightX(), 0.15);
  }

  @Override
  public Trigger isRobotRelative() {
    return m_controller.rightBumper();
  }
}

package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Driver implements DriverInterface {
  private final CommandPS4Controller m_controller;

  public PS4Driver(int port) {
    m_controller = new CommandPS4Controller(port);
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
    return -MathUtil.applyDeadband(m_controller.getRightX(), 0.15);
  }

  @Override
  public Trigger isRobotRelative() {
    return m_controller.R1();
  }
}

package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualJoystickDriver implements DriverInterface {
  private final CommandJoystick m_leftJoystick;
  private final CommandJoystick m_rightJoystick;

  public DualJoystickDriver(int leftPort, int rightPort) {
    m_leftJoystick = new CommandJoystick(leftPort);
    m_rightJoystick = new CommandJoystick(rightPort);
  }

  @Override
  public double getTranslationalMedialPercent() {
    return MathUtil.applyDeadband(-m_leftJoystick.getY(), 0.15);
  }

  @Override
  public double getTranslationalLateralPercent() {
    return MathUtil.applyDeadband(m_leftJoystick.getX(), 0.15);
  }

  @Override
  public double getRotationalPercent() {
    return -MathUtil.applyDeadband(m_rightJoystick.getX(), 0.15);
  }

  @Override
  public Trigger isRobotRelative() {
    return m_leftJoystick.button(1);
  }
}

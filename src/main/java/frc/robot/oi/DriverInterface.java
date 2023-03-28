package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverInterface {
  public double getTranslationalMedialPercent();

  public double getTranslationalLateralPercent();

  public double getRotationalPercent();

  public Trigger isRobotRelative();
}

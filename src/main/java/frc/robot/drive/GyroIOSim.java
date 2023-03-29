package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.robot.constants.Constants;

public class GyroIOSim implements GyroIO {
  private final AnalogGyroSim m_gyro = new AnalogGyroSim(1);

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.YawPositionRad = m_gyro.getAngle();
    inputs.YawRateRadPerSecond = m_gyro.getRate();
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromRadians(m_gyro.getAngle());
  }

  @Override
  public void resetHeading() {
    m_gyro.setAngle(0);
  }

  @Override
  public void incrementHeading(double omegaRadiansPerSecond) {
    m_gyro.setAngle(m_gyro.getAngle() + omegaRadiansPerSecond * Constants.loopPeriodSecs);
  }
}

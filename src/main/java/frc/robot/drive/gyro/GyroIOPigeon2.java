package frc.robot.drive.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_gyro;

  public GyroIOPigeon2(int gyroId) {
    m_gyro = new Pigeon2(gyroId);
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.RollPositionRad = Math.toRadians(m_gyro.getRoll().refresh().getValue());
    inputs.PitchPositionRad = Math.toRadians(m_gyro.getPitch().refresh().getValue());
    inputs.YawPositionRad = Math.toRadians(m_gyro.getYaw().refresh().getValue());

    inputs.RollRateRadPerSecond = Math.toRadians(m_gyro.getAngularVelocityX().refresh().getValue());
    inputs.PitchRateRadPerSecond =
        Math.toRadians(m_gyro.getAngularVelocityY().refresh().getValue());
    inputs.YawRateRadPerSecond = Math.toRadians(m_gyro.getAngularVelocityZ().refresh().getValue());

    inputs.AccelXGForce = m_gyro.getAccelerationX().refresh().getValue();
    inputs.AccelYGForce = m_gyro.getAccelerationY().refresh().getValue();
    inputs.AccelZGForce = m_gyro.getAccelerationZ().refresh().getValue();
  }

  @Override
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void resetHeading() {
    m_gyro.reset();
  }
}

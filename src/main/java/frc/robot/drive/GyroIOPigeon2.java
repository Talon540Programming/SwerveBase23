package frc.robot.drive;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO {
    private final WPI_Pigeon2 m_gyro;

    private final double[] xyzDegreesPerSecond = new double[3];
    private final short[] xyzAccelData = new short[3];

    public GyroIOPigeon2(int gyroId) {
        m_gyro = new WPI_Pigeon2(gyroId);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        m_gyro.getRawGyro(xyzDegreesPerSecond);
        m_gyro.getBiasedAccelerometer(xyzAccelData);

        inputs.Connected = m_gyro.getLastError().equals(ErrorCode.OK);

        inputs.GyroPositionYawRad = Math.toRadians(m_gyro.getYaw());
        inputs.GyroPositionPitchRad = Math.toRadians(m_gyro.getPitch());
        inputs.GyroPositionRollRad = Math.toRadians(m_gyro.getRoll());

        inputs.GyroRateYawRadPerSecond = Math.toRadians(xyzDegreesPerSecond[2]);
        inputs.GyroRatePitchRadPerSecond = Math.toRadians(xyzDegreesPerSecond[1]);
        inputs.GyroRateRollRadPerSecond = Math.toRadians(xyzDegreesPerSecond[0]);

        inputs.AccelXGForce = (double) xyzAccelData[0] / (1 << 14);
        inputs.AccelYGForce = (double) xyzAccelData[1] / (1 << 14);
        inputs.AccelZGForce = (double) xyzAccelData[2] / (1 << 14);
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

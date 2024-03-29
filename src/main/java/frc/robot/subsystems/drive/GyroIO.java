package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d rollPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d yawPosition = new Rotation2d();

    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;

    public double accelX = 0.0;
    public double accelY = 0.0;
    public double accelZ = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}

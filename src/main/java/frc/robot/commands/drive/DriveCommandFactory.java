package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.PoseEstimator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveCommandFactory {
  private static final LoggedDashboardChooser<Double> xCoefficient =
      new LoggedDashboardChooser<>("Drive X Speed Limiter");
  private static final LoggedDashboardChooser<Double> yCoefficient =
      new LoggedDashboardChooser<>("Drive Y Speed Limiter");
  private static final LoggedDashboardChooser<Double> omegaCoefficient =
      new LoggedDashboardChooser<>("Drive Omega Speed Limiter");

  static {
    xCoefficient.addDefaultOption("Default (100%)", 1.0);
    xCoefficient.addOption("Fast (70%)", 0.7);
    xCoefficient.addOption("Medium (30%)", 0.3);
    xCoefficient.addOption("Slow (15%)", 0.15);

    yCoefficient.addDefaultOption("Default (100%)", 1.0);
    yCoefficient.addOption("Fast (70%)", 0.7);
    yCoefficient.addOption("Medium (30%)", 0.3);
    yCoefficient.addOption("Slow (15%)", 0.15);

    omegaCoefficient.addDefaultOption("Default (100%)", 1.0);
    omegaCoefficient.addOption("Fast (70%)", 0.7);
    omegaCoefficient.addOption("Medium (30%)", 0.3);
    omegaCoefficient.addOption("Slow (15%)", 0.15);
  }

  public static Command joystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double deadband) {
    if (deadband >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid Deadband Configured. Deadband value should be (0.0, 1.0), supplied: %f",
              deadband));
    }

    return Commands.run(
        () -> {
          double x_val = xSupplier.getAsDouble() * xCoefficient.get();
          double y_val = ySupplier.getAsDouble() * yCoefficient.get();
          double omega_val = omegaSupplier.getAsDouble() * omegaCoefficient.get();

          double x =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(x_val, deadband), 2), 1.0), x_val);
          double y =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(y_val, deadband), 2), 1.0), y_val);
          double omega =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(omega_val, deadband), 2), 1.0),
                  omega_val);

          driveBase.runVelocity(toFieldRelative(x, y, omega));
        },
        driveBase);
  }

  public static Command sprintJoystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier sprintSupplier,
      double maxNonSprintSpeed,
      double deadband) {
    if (maxNonSprintSpeed >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid non-sprint max speed. Max Sprint speed should be (0.0, 1.0), supplied %f",
              maxNonSprintSpeed));
    }
    if (deadband >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid Deadband Configured. Deadband value should be (0.0, 1.0), supplied: %f",
              deadband));
    }

    return Commands.run(
        () -> {
          double x_val = xSupplier.getAsDouble() * xCoefficient.get() * maxNonSprintSpeed;
          double y_val = ySupplier.getAsDouble() * yCoefficient.get() * maxNonSprintSpeed;
          double omega_val = omegaSupplier.getAsDouble() * omegaCoefficient.get();

          double sprintVal = sprintSupplier.getAsBoolean() ? 0.5 : 0.0;
          double x =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(x_val, deadband), 2) + sprintVal, 1.0),
                  x_val);
          double y =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(y_val, deadband), 2) + sprintVal, 1.0),
                  y_val);
          double omega =
              Math.copySign(
                  Math.min(Math.pow(MathUtil.applyDeadband(omega_val, deadband), 2), 1.0),
                  omega_val);

          driveBase.runVelocity(toFieldRelative(x, y, omega));
        },
        driveBase);
  }

  private static ChassisSpeeds toFieldRelative(double x, double y, double omega) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        x * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        y * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        omega * Constants.Drivetrain.kMaxAngularVelocityRadiansPerSecond,
        PoseEstimator.getInstance().getPose().getRotation());
  }

  private static ChassisSpeeds toRobotRelative(double x, double y, double omega) {
    return new ChassisSpeeds(
        x * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        y * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        omega * Constants.Drivetrain.kMaxAngularVelocityRadiansPerSecond);
  }
}

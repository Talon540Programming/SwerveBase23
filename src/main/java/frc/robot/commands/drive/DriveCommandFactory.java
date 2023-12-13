package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.PoseEstimator;
import java.util.function.DoubleSupplier;

public class DriveCommandFactory {
  public static Command joystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double deadband) {
    return Commands.run(
        () -> {
          double x_val = xSupplier.getAsDouble();
          double y_val = ySupplier.getAsDouble();
          double omega_val = omegaSupplier.getAsDouble();

          double x = Math.copySign(Math.pow(MathUtil.applyDeadband(x_val, deadband), 2), x_val);
          double y = Math.copySign(Math.pow(MathUtil.applyDeadband(y_val, deadband), 2), y_val);
          double omega =
              Math.copySign(Math.pow(MathUtil.applyDeadband(omega_val, deadband), 2), omega_val);

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

package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PoseEstimator;
import frc.robot.constants.Constants;
import frc.robot.drive.gyro.GyroIO;
import frc.robot.drive.gyro.GyroInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class DriveBase extends SubsystemBase {
  private final SwerveModuleIO[] m_moduleIOs;
  private final SwerveModuleInputsAutoLogged[] m_moduleInputs =
      new SwerveModuleInputsAutoLogged[] {
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged()
      };

  private final GyroIO m_gyroIO;
  private final GyroInputsAutoLogged m_gyroInputs = new GyroInputsAutoLogged();

  public DriveBase(
      GyroIO gyro,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    m_moduleIOs = new SwerveModuleIO[] {frontLeft, frontRight, backLeft, backRight};
    m_gyroIO = gyro;

    m_gyroIO.resetHeading();

    for (SwerveModuleIO module : m_moduleIOs) {
      module.resyncEncoders();
    }

    PoseEstimator.createInstance(
        Constants.Drivetrain.kKinematics, m_gyroIO.getHeading(), getModulePositions());
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      m_moduleIOs[i].updateInputs(m_moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + i, m_moduleInputs[i]);
    }

    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", m_gyroInputs);

    // Update PoseEstimator with drive data
    PoseEstimator.getInstance().update(m_gyroIO.getHeading(), getModulePositions());
    Logger.getInstance()
        .recordOutput("EstimatedPose", PoseEstimator.getInstance().getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    m_gyroIO.incrementHeading(getChassisSpeeds().omegaRadiansPerSecond * Constants.loopPeriodSecs);
  }

  public Rotation2d getHeading() {
    return m_gyroIO.getHeading();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.Drivetrain.kKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Set the drivetrain modules based on a series of speeds and spinning about a non-center of the
   * robot given as a {@link Translation2d} object and following its conventions.
   *
   * @param xSpeed Horizontal speed to derive module states in meters per second
   * @param ySpeed Vertical speed to derive module states in meters per second
   * @param rotationalSpeed Rotational speed in radians per second the drivetrain spins in
   * @param driveMode Control input method to derive module states from: {@code DriveMode.FIELD} or
   *     {@code DriveMode.DRIVER} (default FIELD)
   */
  public final void setFromForces(
      double xSpeed,
      double ySpeed,
      double rotationalSpeed,
      Translation2d centerOfRobot,
      DriveMode driveMode) {

    setModuleStates(
        Constants.Drivetrain.kKinematics.toSwerveModuleStates(
            switch (driveMode) {
              case kRobot -> new ChassisSpeeds(xSpeed, ySpeed, rotationalSpeed);
              case kField -> ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed, ySpeed, rotationalSpeed, getHeading());
            },
            centerOfRobot));
  }

  /**
   * Control method to convert input stuff to drive stuff for all the modules
   *
   * @param xSpeed Horizontal speed to derive module states in meters per second
   * @param ySpeed Vertical speed to derive module states in meters per second
   * @param rotationalSpeed Rotational speed in radians per second the drivetrain spins in
   * @param driveMode Control input method to derive module states from: {@code DriveMode.FIELD} or
   *     {@code DriveMode.DRIVER}
   */
  public final void setFromForces(
      double xSpeed, double ySpeed, double rotationalSpeed, DriveMode driveMode) {
    setFromForces(xSpeed, ySpeed, rotationalSpeed, new Translation2d(), driveMode);
  }

  /** Apply module states to modules based on a ChassisSpeed object */
  public final void setChassisSpeed(ChassisSpeeds chassisSpeed) {
    setModuleStates(Constants.Drivetrain.kKinematics.toSwerveModuleStates(chassisSpeed));
  }

  /**
   * Apply module states to modules based on an array of module states.
   *
   * @implNote MODULES MUST BE GIVEN IN ORDER: FRONT LEFT -> FRONT RIGHT -> BACK LEFT -> BACK RIGHT
   */
  public final void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Drivetrain.kMaxVelocityMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      m_moduleIOs[i].setState(states[i]);
    }
  }

  public void stop() {
    for (SwerveModuleIO module : m_moduleIOs) {
      module.stop();
    }
  }

  public void stopWithX() {
    double[] stopAngles =
        new double[] {-Math.PI / 4.0, Math.PI / 4.0, Math.PI / 4.0, -Math.PI / 4.0};

    for (int i = 0; i < m_moduleIOs.length; i++) {
      m_moduleIOs[i].setState(new SwerveModuleState(0, Rotation2d.fromRadians(stopAngles[i])));
    }
  }

  /** Return module states in order of kinematic initialization from modules */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_moduleIOs[0].getState(),
      m_moduleIOs[1].getState(),
      m_moduleIOs[2].getState(),
      m_moduleIOs[3].getState()
    };
  }

  /** Return module positions in order of kinematic initialization from modules */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_moduleIOs[0].getPosition(),
      m_moduleIOs[1].getPosition(),
      m_moduleIOs[2].getPosition(),
      m_moduleIOs[3].getPosition()
    };
  }

  public enum DriveMode {
    kRobot,
    kField
  }
}

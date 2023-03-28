package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight,
      GyroIO gyro) {
    m_moduleIOs = new SwerveModuleIO[] {frontLeft, frontRight, backLeft, backRight};
    m_gyroIO = gyro;

    for(SwerveModuleIO module : m_moduleIOs) {
      module.resyncEncoders();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      m_moduleIOs[i].updateInputs(m_moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + i, m_moduleInputs[i]);
    }

    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", m_gyroInputs);
  }

  /** Return module states in order of kinematic initialization from modules */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            this.m_moduleIOs[0].getState(),
            this.m_moduleIOs[1].getState(),
            this.m_moduleIOs[2].getState(),
            this.m_moduleIOs[3].getState()
    };
  }

  /** Return module positions in order of kinematic initialization from modules */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            this.m_moduleIOs[0].getPosition(),
            this.m_moduleIOs[1].getPosition(),
            this.m_moduleIOs[2].getPosition(),
            this.m_moduleIOs[3].getPosition()
    };
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

    this.setFromModuleStates(
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

  /**
   * (NO SPIN) Control method to convert input stuff to drive stuff for all the modules without
   * applying a spin value to the drivetrain. Think... tank drive
   *
   * @param xSpeed Horizontal speed to derive module states in meters per second
   * @param ySpeed Vertical speed to derive module states in meters per second
   * @param driveMode Control input method to derive module states from: {@code DriveMode.FIELD} or
   *     {@code DriveMode.DRIVER} (default FIELD)
   */
  public final void setFromForces(double xSpeed, double ySpeed, DriveMode driveMode) {
    this.setFromForces(xSpeed, ySpeed, 0, driveMode);
  }

  /** Apply module states to modules based on a ChassisSpeed object */
  public final void setFromChassisSpeed(ChassisSpeeds chassisSpeed) {
    this.setFromModuleStates(Constants.Drivetrain.kKinematics.toSwerveModuleStates(chassisSpeed));
  }

  /**
   * Apply module states to modules based on an array of module states.
   *
   * @implNote MODULES MUST BE GIVEN IN ORDER: FRONT LEFT -> FRONT RIGHT -> BACK LEFT -> BACK RIGHT
   */
  public final void setFromModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.kMaxVelocityMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      m_moduleIOs[i].setState(states[i]);
    }
  }

  public void stop() {
    for(SwerveModuleIO module : m_moduleIOs) {
      module.stop();
    }
  }

  public enum DriveMode {
    kRobot,
    kField
  }
}

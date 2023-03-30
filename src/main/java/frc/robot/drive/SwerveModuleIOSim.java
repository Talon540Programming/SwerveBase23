package frc.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private final SwerveModuleState m_currentState = new SwerveModuleState();
  private final SwerveModulePosition m_currentPosition = new SwerveModulePosition();

  @Override
  public SwerveModuleState getState() {
    return m_currentState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return m_currentPosition;
  }

  @Override
  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, m_currentState.angle);

    m_currentState.speedMetersPerSecond = state.speedMetersPerSecond;
    m_currentState.angle = state.angle;
  }

  @Override
  public void stop() {
    m_currentState.speedMetersPerSecond = 0;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // Update Position
    m_currentPosition.distanceMeters +=
        m_currentState.speedMetersPerSecond * Constants.loopPeriodSecs;
    m_currentPosition.angle = m_currentState.angle;

    inputs.DistanceTraveledMeters = m_currentPosition.distanceMeters;
    inputs.VelocityMetersPerSec = m_currentState.speedMetersPerSecond;

    inputs.AbsolutePositionRad = m_currentPosition.angle.getRadians();
  }
}

package frc.lib;

/**
 * Represents an IO interface that can control a {@link
 * org.littletonrobotics.junction.inputs.LoggableInputs}.
 *
 * @param <T> Loggable inputs that are updated.
 */
public interface LoggedIO<T> {

  /**
   * Update the inputs with the current state of the IO.
   *
   * @param inputs inputs to update.
   */
  default void updateInputs(T inputs) {}
}

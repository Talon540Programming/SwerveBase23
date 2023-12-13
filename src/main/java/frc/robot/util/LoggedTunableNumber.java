package frc.robot.util;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private Double defaultValue = null;
  private Double lastValue = null;
  private LoggedDashboardNumber dashboardNumber;

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (this.defaultValue == null) {
      this.defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (defaultValue == null) {
      throw new IllegalStateException(
          String.format(
              "[LoggedTunableNumber][%s] Hasn't been initialized with a default value. Make sure to call initDefault or use the correct constructor.",
              key));
    }

    return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Checks whether the number has changed since the last time this method was called.
   *
   * @return Whether the value has changed since the last time this method was called
   */
  public boolean hasChanged() {
    var currentVal = get();
    if (lastValue == null || lastValue != currentVal) {
      lastValue = currentVal;
      return true;
    } else {
      return false;
    }
  }
}

package frc.lib;

import edu.wpi.first.wpilibj.RobotBase;
import frc.generated.BuildConstants;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * SparkMax's memory, like most NAND based memory, has a limited number of write commands it can
 * sustain, in order to prevent this from hitting that maximum, we can check if the code was even
 * changed, and if so, burn the new settings to the SparkMax.
 */
public class SparkMaxBurnManager {
  public static final int kConfigurationStatusTimeoutMs = 500;
  private static final String buildDateFile = "/home/lvuser/build-date.txt";
  private static boolean shouldBurn = false;

  private SparkMaxBurnManager() {}

  public static void checkBuildStatus() {
    // TODO, make sure read and write functions work
    if (RobotBase.isSimulation()) {
      shouldBurn = false;
      return;
    }

    File file = new File(buildDateFile);
    if (!file.exists()) {
      // No build date file, burn flash
      shouldBurn = true;
    } else {
      // Read previous build date
      String previousBuildDate = "";
      try {
        previousBuildDate = Files.readString(Paths.get(buildDateFile));
      } catch (IOException error) {
        error.printStackTrace();
      }

      shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
    }

    // Write the current Build Time
    try {
      FileWriter fileWriter = new FileWriter(buildDateFile);
      fileWriter.write(BuildConstants.BUILD_DATE);
      fileWriter.close();
    } catch (IOException error) {
      error.printStackTrace();
    }

    if (shouldBurn) {
      System.out.println(
          "[SparkMaxBurnManager] Build date changed, allowing SparkMaxes to burn flash");
    } else {
      System.out.println(
          "[SparkMaxBurnManager] Build date unchanged, will not burn SparkMax flash");
    }
  }

  public static boolean shouldBurnFlash() {
    return shouldBurn;
  }
}

// 新檔案: frc/robot/lib/FieldUtil.java
package frc.robot.lib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

public final class FieldUtil {

  private FieldUtil() {
    // utility class, 不需要實例化
  }

  public static Translation2d getHubPosition() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return new Translation2d(FieldConstants.redHubX, FieldConstants.redHubY);
    }
    return new Translation2d(FieldConstants.blueHubX, FieldConstants.blueHubY);
  }
}
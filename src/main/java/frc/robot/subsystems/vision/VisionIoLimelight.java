package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstant;
import java.util.function.DoubleSupplier;

public class VisionIoLimelight implements VisionIo {

  private static final DoubleSupplier kZero = () -> 0.0;

  private final String[] cameraNames;
  private final NetworkTableInstance nt;
  private final NetworkTable[] tables;
  private final DoubleSupplier timeSource;
  private final DoubleSupplier yawSupplierDegrees;
  private final DoubleSupplier yawRateSupplierDegrees;

  public VisionIoLimelight(String... cameraNames) {
    this(NetworkTableInstance.getDefault(), Timer::getFPGATimestamp, null, kZero, cameraNames);
  }

  public VisionIoLimelight(DoubleSupplier yawSupplierDegrees, String... cameraNames) {
    this(NetworkTableInstance.getDefault(), Timer::getFPGATimestamp,
        yawSupplierDegrees, kZero, cameraNames);
  }

  /** MegaTag2 在機器人自轉時需要 yawRate 才能補償,賽季版有傳,這裡補上. */
  public VisionIoLimelight(
      DoubleSupplier yawSupplierDegrees,
      DoubleSupplier yawRateSupplierDegrees,
      String... cameraNames) {
    this(NetworkTableInstance.getDefault(), Timer::getFPGATimestamp,
        yawSupplierDegrees, yawRateSupplierDegrees, cameraNames);
  }

  VisionIoLimelight(
      NetworkTableInstance nt,
      DoubleSupplier timeSource,
      DoubleSupplier yawSupplierDegrees,
      DoubleSupplier yawRateSupplierDegrees,
      String... cameraNames) {
    if (cameraNames.length == 0) {
      throw new IllegalArgumentException("VisionIOLimelight requires at least one camera name");
    }
    this.cameraNames = cameraNames.clone();
    this.nt = nt;
    this.timeSource = timeSource;
    this.yawSupplierDegrees = yawSupplierDegrees;
    this.yawRateSupplierDegrees = yawRateSupplierDegrees;
    this.tables = new NetworkTable[cameraNames.length];
    for (int i = 0; i < cameraNames.length; i++) {
      this.tables[i] = nt.getTable(cameraNames[i]);
    }
  }

  @Override
  public void updateInputs(VisionIoInputs inputs) {
    if (inputs.cameras.length != cameraNames.length) {
      inputs.cameras = new CameraInputs[cameraNames.length];
      for (int i = 0; i < cameraNames.length; i++) {
        inputs.cameras[i] = new CameraInputs();
      }
    }
    // 先把姿態推給所有相機再一次 flush,MegaTag2 下一輪解算才拿得到最新的 yaw。
    // 少了 flush,yaw 要等下一個 NT 週期才送出,LL 會用過期的角度解 botpose_orb。
    if (yawSupplierDegrees != null) {
      double[] orientation = {
          yawSupplierDegrees.getAsDouble(), yawRateSupplierDegrees.getAsDouble(), 0, 0, 0, 0 };
      for (NetworkTable table : tables) {
        table.getEntry("robot_orientation_set").setDoubleArray(orientation);
      }
      nt.flush();
    }

    double now = timeSource.getAsDouble();
    for (int i = 0; i < tables.length; i++) {
      updateCamera(tables[i], inputs.cameras[i], now);
    }
  }

  private void updateCamera(NetworkTable table, CameraInputs cam, double now) {
    boolean seesTarget = table.getEntry("tv").getDouble(0.0) > 0.5;
    cam.seesTarget = seesTarget;

    FiducialObservation[] fiducials = parseFiducials(table.getEntry("rawfiducials").getDoubleArray(new double[0]));
    cam.fiducials = fiducials;
    double minAmbiguity = minAmbiguity(fiducials);

    double[] mt1 = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    double[] mt2 = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

    cam.megatagPoseEstimate = parseEstimate(mt1, fiducials, minAmbiguity, now, seesTarget);
    cam.megatag2PoseEstimate = parseEstimate(mt2, fiducials, minAmbiguity, now, seesTarget);
    cam.megatagCount = tagCount(mt1);
    cam.megatag2Count = tagCount(mt2);

    cam.pose3d = parsePose3d(isValidBotpose(mt2) ? mt2 : mt1);

    double[] std = new double[6];
    fillStdDevs(std, 0, mt1);
    fillStdDevs(std, 3, mt2);
    cam.standardDeviations = std;
  }

  private MegatagPoseEstimate parseEstimate(
      double[] p, FiducialObservation[] fiducials, double minAmbiguity, double now, boolean seesTarget) {
    if (!seesTarget || !isValidBotpose(p)) {
      return null;
    }
    int count = tagCount(p);
    Pose2d pose = new Pose2d(p[VisionConstant.botposeX], p[VisionConstant.botposeY],
        Rotation2d.fromDegrees(p[VisionConstant.botposeYaw]));
    double latencyMs = p[VisionConstant.botposeLatency];
    double timestamp = now - latencyMs / 1000.0;
    double avgArea = p[VisionConstant.botposeAvgArea];
    double avgDist = p[VisionConstant.botposeAvgDist];
    double quality = computeQuality(count, avgArea, minAmbiguity);
    return new MegatagPoseEstimate(
        pose, timestamp, latencyMs, avgArea, avgDist, quality, count, fiducialIds(fiducials));
  }

  private static double computeQuality(int tagCount, double avgTagArea, double minAmbiguity) {
    if (tagCount <= 0) {
      return 0.0;
    }
    double ambiguityScore = clamp(1.0 - minAmbiguity, 0.0, 1.0);
    if (tagCount > 1) {
      ambiguityScore = Math.max(ambiguityScore, 0.7);
    }
    double areaScore = clamp(avgTagArea / VisionConstant.areaReference, 0.25, 1.0);
    return clamp(ambiguityScore * areaScore, 0.0, 1.0);
  }

  private static FiducialObservation[] parseFiducials(double[] raw) {
    if (raw.length < VisionConstant.rawfiducialsStride) {
      return new FiducialObservation[0];
    }
    int n = raw.length / VisionConstant.rawfiducialsStride;
    FiducialObservation[] out = new FiducialObservation[n];
    for (int i = 0; i < n; i++) {
      int b = i * VisionConstant.rawfiducialsStride;
      out[i] = new FiducialObservation(
          (int) Math.round(raw[b + VisionConstant.rfId]),
          raw[b + VisionConstant.rfTxnc],
          raw[b + VisionConstant.rfTync],
          raw[b + VisionConstant.rfAmbiguity],
          raw[b + VisionConstant.rfTa]);
    }
    return out;
  }

  /**
   * 沒有 rawfiducials 資料時回傳 0(視為無歧義),而不是最壞的 1.0。
   * 回 1.0 會讓 computeQuality 算出 0 → 量測一律被拒絕,這是「什麼都看不到」的主因之一;
   * 賽季版可用的寫法根本不讀 rawfiducials,這裡讓它退化成同樣的行為.
   */
  private static double minAmbiguity(FiducialObservation[] fiducials) {
    if (fiducials.length == 0) {
      return 0.0;
    }
    double min = 1.0;
    for (FiducialObservation f : fiducials) {
      min = Math.min(min, f.ambiguity());
    }
    return min;
  }

  private static int[] fiducialIds(FiducialObservation[] fiducials) {
    int[] ids = new int[fiducials.length];
    for (int i = 0; i < fiducials.length; i++) {
      ids[i] = fiducials[i].id();
    }
    return ids;
  }

  private static Pose3d parsePose3d(double[] p) {
    if (!isValidBotpose(p)) {
      return new Pose3d();
    }
    return new Pose3d(
        p[VisionConstant.botposeX],
        p[VisionConstant.botposeY],
        p[VisionConstant.botposeZ],
        new Rotation3d(
            Math.toRadians(p[VisionConstant.botposeRoll]),
            Math.toRadians(p[VisionConstant.botposePitch]),
            Math.toRadians(p[VisionConstant.botposeYaw])));
  }

  private static void fillStdDevs(double[] out, int offset, double[] p) {
    int count = tagCount(p);
    if (count <= 0 || !isValidBotpose(p)) {
      out[offset] = out[offset + 1] = out[offset + 2] = VisionConstant.untrustedStd;
      return;
    }
    double dist = p[VisionConstant.botposeAvgDist];
    double factor = (dist * dist) / count;
    out[offset] = VisionConstant.baseTranslationStd * factor;
    out[offset + 1] = VisionConstant.baseTranslationStd * factor;
    out[offset + 2] = Math.toRadians(5) * factor;
  }

  private static int tagCount(double[] p) {
    if (p.length < VisionConstant.botposeLength) {
      return 0;
    }
    return Math.max(0, (int) Math.round(p[VisionConstant.botposeTagCount]));
  }

  private static boolean isValidBotpose(double[] p) {
    return p.length >= VisionConstant.botposeLength && tagCount(p) > 0;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}

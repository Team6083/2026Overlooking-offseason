package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstant;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

  private static final double kPoseTrustTimeoutSec = 0.5;

  private final VisionIo io;
  private final VisionIo.VisionIoInputs inputs = new VisionIo.VisionIoInputs();
  private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> poseConsumer;
  private final Supplier<Pose2d> poseSupplier;
  private double lastAcceptedTimestamp = Double.NEGATIVE_INFINITY;

  private final Map<Integer, CameraPublishers> cameraPublishers = new HashMap<>();

  public VisionSubsystem(
      VisionIo io,
      Supplier<Pose2d> poseSupplier,
      TriConsumer<Pose2d, Double, Matrix<N3, N1>> poseConsumer) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.poseConsumer = poseConsumer;
  }

  public VisionSubsystem(
      VisionIo io,
      Supplier<Pose2d> poseSupplier,
      BiConsumer<Pose2d, Double> poseConsumer) {
    this(io, poseSupplier, (pose, ts, stdDevs) -> poseConsumer.accept(pose, ts));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs();
  }

  private void processInputs() {
    int totalAccepted = 0;
    int totalRejected = 0;

    for (int i = 0; i < inputs.cameras.length; i++) {
      VisionIo.CameraInputs camera = inputs.cameras[i];

      // 先無條件發布原始量測,被拒絕的也要看得到 —— 否則相機 offset 有問題時
      // 量測會被 quality 門檻擋掉,AdvantageScope 上什麼都不會出現,反而查不出原因。
      publishRawPoses(i, camera);

      if (!camera.seesTarget) {
        continue;
      }

      MegatagPoseEstimate estimate = selectBestEstimate(camera);
      if (estimate == null) {
        continue;
      }

      if (shouldReject(estimate)) {
        totalRejected++;
        continue;
      }

      // 照賽季版可用的寫法:旋轉直接沿用底盤自己的朝向,只採用 vision 的平移。
      // MegaTag2 的旋轉本來就是我們透過 robot_orientation_set 餵進去的 gyro,
      // 把它融回 estimator 等於 gyro 校正 gyro。theta 的 stdDev 再給極大值做第二層保險。
      Pose2d fusedPose = new Pose2d(
          estimate.fieldToRobot().getTranslation(), poseSupplier.get().getRotation());
      poseConsumer.accept(fusedPose, estimate.timestampSeconds(), stdDevsFor(estimate));
      totalAccepted++;
      lastAcceptedTimestamp = Timer.getFPGATimestamp();

      logCamera(i, estimate);
    }

    SmartDashboard.putNumber("Vision/AcceptedMeasurements", totalAccepted);
    SmartDashboard.putNumber("Vision/RejectedMeasurements", totalRejected);
    SmartDashboard.putBoolean("Vision/PoseTrusted", isPoseTrusted());
  }

  /** True when a vision measurement was accepted recently enough to trust the fused pose. */
  public boolean isPoseTrusted() {
    return Timer.getFPGATimestamp() - lastAcceptedTimestamp < kPoseTrustTimeoutSec;
  }

  private MegatagPoseEstimate selectBestEstimate(VisionIo.CameraInputs camera) {
    if (camera.megatag2PoseEstimate != null && camera.megatag2PoseEstimate.isValid()) {
      return camera.megatag2PoseEstimate;
    }
    if (camera.megatagPoseEstimate != null && camera.megatagPoseEstimate.isValid()) {
      return camera.megatagPoseEstimate;
    }
    return null;
  }

  /**
   * 依平均 tag 距離加權,照賽季版的公式 min(0.4 + 距離 × 0.6, 5.0)。
   * 旋轉一律給極大值 —— vision 的旋轉不參與融合.
   */
  private static Matrix<N3, N1> stdDevsFor(MegatagPoseEstimate estimate) {
    double trust = Math.min(
        VisionConstant.trustBase + estimate.avgTagDist() * VisionConstant.trustPerMeter,
        VisionConstant.trustMax);
    if (estimate.isMultiTag()) {
      trust *= VisionConstant.multiTagTrustFactor;
    }
    return VecBuilder.fill(trust, trust, VisionConstant.untrustedRotationStd);
  }

  private boolean shouldReject(MegatagPoseEstimate estimate) {
    if (estimate.quality() < VisionConstant.minQualityThreshold) {
      return true;
    }
    return false;
  }

  /**
   * 把兩種 MegaTag 的原始 botpose 發成 Pose2d/Pose3d struct 陣列,AdvantageScope 才畫得出來。
   * 用陣列而不是單一 struct:沒有量測時發空陣列,場地圖上的殘影會消失,不會凍結在最後一次的位置.
   */
  private void publishRawPoses(int index, VisionIo.CameraInputs camera) {
    CameraPublishers pubs = publishersFor(index);
    pubs.megatag1().set(toArray(camera.megatagPoseEstimate));
    pubs.megatag2().set(toArray(camera.megatag2PoseEstimate));
    pubs.botPose3d().set(
        camera.seesTarget ? new Pose3d[] { camera.pose3d } : new Pose3d[0]);
    // 先清掉,這個 loop 真的被接受的話 logCamera() 會再填回去。
    pubs.accepted().set(new Pose2d[0]);
  }

  private static Pose2d[] toArray(MegatagPoseEstimate estimate) {
    if (estimate == null || !estimate.isValid()) {
      return new Pose2d[0];
    }
    return new Pose2d[] { estimate.fieldToRobot() };
  }

  private CameraPublishers publishersFor(int index) {
    return cameraPublishers.computeIfAbsent(index, i -> {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision/Camera" + i);
      return new CameraPublishers(
          table.getStructArrayTopic("MegaTag1Pose", Pose2d.struct).publish(),
          table.getStructArrayTopic("MegaTag2Pose", Pose2d.struct).publish(),
          table.getStructArrayTopic("BotPose3d", Pose3d.struct).publish(),
          table.getStructArrayTopic("AcceptedPose", Pose2d.struct).publish());
    });
  }

  private record CameraPublishers(
      StructArrayPublisher<Pose2d> megatag1,
      StructArrayPublisher<Pose2d> megatag2,
      StructArrayPublisher<Pose3d> botPose3d,
      StructArrayPublisher<Pose2d> accepted) {
  }

  private void logCamera(int index, MegatagPoseEstimate estimate) {
    publishersFor(index).accepted().set(new Pose2d[] { estimate.fieldToRobot() });
    String prefix = "Vision/Camera" + index + "/";
    SmartDashboard.putNumber(prefix + "Quality", estimate.quality());
    SmartDashboard.putBoolean(prefix + "MultiTag", estimate.isMultiTag());
    SmartDashboard.putNumber(prefix + "TagCount", estimate.tagCount());
    SmartDashboard.putNumber(prefix + "AvgTagArea", estimate.avgTagArea());
    SmartDashboard.putNumber(prefix + "PoseX", estimate.fieldToRobot().getX());
    SmartDashboard.putNumber(prefix + "PoseY", estimate.fieldToRobot().getY());
    SmartDashboard.putNumber(prefix + "PoseRot", estimate.fieldToRobot().getRotation().getDegrees());
  }

  @FunctionalInterface
  public interface TriConsumer<A, B, C> {
    void accept(A a, B b, C c);
  }
}

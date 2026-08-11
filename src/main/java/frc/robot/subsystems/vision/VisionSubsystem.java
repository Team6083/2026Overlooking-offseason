package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstant;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIo io;
  private final VisionIo.VisionIoInputs inputs = new VisionIo.VisionIoInputs();
  private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> poseConsumer;

  public VisionSubsystem(
      VisionIo io,
      Supplier<Pose2d> poseSupplier,
      TriConsumer<Pose2d, Double, Matrix<N3, N1>> poseConsumer) {
    this.io = io;
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

      Matrix<N3, N1> stdDevs = estimate.isMultiTag() ? VisionConstant.multiTagStdDevs : VisionConstant.singleTagStdDevs;
      poseConsumer.accept(estimate.fieldToRobot(), estimate.timestampSeconds(), stdDevs);
      totalAccepted++;

      logCamera(i, estimate);
    }

    SmartDashboard.putNumber("Vision/AcceptedMeasurements", totalAccepted);
    SmartDashboard.putNumber("Vision/RejectedMeasurements", totalRejected);
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

  private boolean shouldReject(MegatagPoseEstimate estimate) {
    if (estimate.quality() < VisionConstant.minQualityThreshold) {
      return true;
    }
    return false;
  }

  private void logCamera(int index, MegatagPoseEstimate estimate) {
    String prefix = "Vision/Camera" + index + "/";
    SmartDashboard.putNumber(prefix + "Quality", estimate.quality());
    SmartDashboard.putBoolean(prefix + "MultiTag", estimate.isMultiTag());
    SmartDashboard.putNumber(prefix + "TagCount", estimate.fiducialIds().length);
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

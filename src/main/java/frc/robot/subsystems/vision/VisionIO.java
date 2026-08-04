package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

  class CameraInputs {
    public boolean seesTarget = false;

    public MegatagPoseEstimate megatagPoseEstimate = null;

    public MegatagPoseEstimate megatag2PoseEstimate = null;

    public int megatagCount = 0;

    public int megatag2Count = 0;

    public Pose3d pose3d = new Pose3d();

    public double[] standardDeviations = new double[6];

    public FiducialObservation[] fiducials = new FiducialObservation[0];
  }

  class VisionIOInputs {
    public CameraInputs[] cameras = new CameraInputs[0];
  }

  void updateInputs(VisionIOInputs inputs);
}
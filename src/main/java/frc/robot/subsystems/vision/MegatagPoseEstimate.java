package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record MegatagPoseEstimate(
    Pose2d fieldToRobot,
    double timestampSeconds,
    double latency,
    double avgTagArea,
    double quality,
    int[] fiducialIds)
    implements StructSerializable {

  public MegatagPoseEstimate {
    if (fieldToRobot == null)
      fieldToRobot = new Pose2d();
    if (fiducialIds == null)
      fiducialIds = new int[0];
  }

  public boolean isMultiTag() {
    return fiducialIds.length > 1;
  }

  public boolean isValid() {
    return fiducialIds.length > 0 && quality > 0.0;
  }

  public static final Struct<MegatagPoseEstimate> struct = new MegatagPoseEstimateStruct();

  public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {

    @Override
    public Class<MegatagPoseEstimate> getTypeClass() {
      return MegatagPoseEstimate.class;
    }

    @Override
    public String getTypeName() {
      return "MegatagPoseEstimate";
    }

    @Override
    public String getTypeString() {
      return "record:MegatagPoseEstimate";
    }

    @Override
    public int getSize() {
      return Pose2d.struct.getSize() + 4 * Double.BYTES;
    }

    @Override
    public String getSchema() {
      return "Pose2d fieldToRobot;double timestampSeconds;double latency;double avgTagArea;double quality";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] { Pose2d.struct };
    }

    @Override
    public MegatagPoseEstimate unpack(ByteBuffer bb) {
      return new MegatagPoseEstimate(
          Pose2d.struct.unpack(bb),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          new int[0]);
    }

    @Override
    public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToRobot());
      bb.putDouble(value.timestampSeconds());
      bb.putDouble(value.latency());
      bb.putDouble(value.avgTagArea());
      bb.putDouble(value.quality());
    }
  }
}
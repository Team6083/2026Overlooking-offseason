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
    double avgTagDist,
    double quality,
    int tagCount,
    int[] fiducialIds)
    implements StructSerializable {

  public MegatagPoseEstimate {
    if (fieldToRobot == null) {
      fieldToRobot = new Pose2d();
    }
    if (fiducialIds == null) {
      fiducialIds = new int[0];
    }
  }

  /**
   * 用 botpose 回報的 tagCount,不是 rawfiducials 的長度 —— 相機可能看到 3 顆 tag 但只用 1 顆
   * 解算,那時套用 multi-tag 的高信任度會過度信任.
   */
  public boolean isMultiTag() {
    return tagCount > 1;
  }

  /**
   * 只看 botpose 自己回報的 tagCount。賽季版可用的寫法就是只靠 tv + botpose 陣列,
   * 不依賴 rawfiducials —— 那個 key 沒資料時整批量測會被靜默丟掉.
   */
  public boolean isValid() {
    return tagCount > 0;
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
      return Pose2d.struct.getSize() + 5 * Double.BYTES + Integer.BYTES;
    }

    @Override
    public String getSchema() {
      return "Pose2d fieldToRobot;double timestampSeconds;double latency;double avgTagArea;double avgTagDist;double quality;int32 tagCount";
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
          bb.getDouble(),
          bb.getInt(),
          new int[0]);
    }

    @Override
    public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToRobot());
      bb.putDouble(value.timestampSeconds());
      bb.putDouble(value.latency());
      bb.putDouble(value.avgTagArea());
      bb.putDouble(value.avgTagDist());
      bb.putDouble(value.quality());
      bb.putInt(value.tagCount());
    }
  }
}
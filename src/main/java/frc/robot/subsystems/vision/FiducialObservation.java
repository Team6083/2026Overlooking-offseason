package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area)
    implements StructSerializable {

  public static final Struct<FiducialObservation> struct = new Struct<>() {
    @Override
    public Class<FiducialObservation> getTypeClass() {
      return FiducialObservation.class;
    }

    @Override
    public String getTypeName() {
      return "FiducialObservation";
    }

    @Override
    public String getTypeString() {
      return "record:FiducialObservation";
    }

    @Override
    public int getSize() {
      return Integer.BYTES + 4 * Double.BYTES;
    }

    @Override
    public String getSchema() {
      return "int id;double txnc;double tync;double ambiguity;double area";
    }

    @Override
    public FiducialObservation unpack(ByteBuffer bb) {
      return new FiducialObservation(
          bb.getInt(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, FiducialObservation value) {
      bb.putInt(value.id());
      bb.putDouble(value.txnc());
      bb.putDouble(value.tync());
      bb.putDouble(value.ambiguity());
      bb.putDouble(value.area());
    }
  };
}
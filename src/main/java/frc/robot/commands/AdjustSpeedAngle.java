package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.lib.shooting.ShotTable;
import frc.robot.lib.shooting.ShotTable.Candidate;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AdjustSpeedAngle extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final SwerveDrive swerveDrive;
  private final ShotTable shotTable;

  private double targetVelocity;
  private double targetAngle;

  public AdjustSpeedAngle(ShooterSubsystem shooterSubsystem,
      AngleSubsystem angleSubsystem,
      SwerveDrive swerveDrive,
      ShotTable shotTable) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.swerveDrive = swerveDrive;
    this.shotTable = shotTable;
    addRequirements(shooterSubsystem, angleSubsystem);
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    Distance dis = Meters.of(robotPos.getDistance(getHubPosition()));
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);

    if (inTrench) {
      targetAngle = AngleConstants.angleMotorMinAngle;
      // trench 中維持目前轉速不變，避免顛簸期間亂調(沿用目前 targetVelocity)
    } else {
      Candidate solution = shotTable.pickClosestVelocity(
          dis.in(Centimeters), shooterSubsystem.getShooterVelocity());

      if (solution != null) {
        targetAngle = solution.angleDeg();
        targetVelocity = solution.velocityRpm();
      } else {
        // 超出實測網格範圍(太近或太遠)的 fallback：維持上一次的值，不要亂噴
        SmartDashboard.putBoolean("shooter/outOfTableRange", true);
      }
    }

    shooterSubsystem.shootCmd(targetVelocity);
    angleSubsystem.angleSync(targetAngle);

    SmartDashboard.putNumber("shooterDistance", dis.in(Centimeters));
    SmartDashboard.putNumber("shooterTargetAngle", targetAngle);
    SmartDashboard.putNumber("shooterTargetVelocity", targetVelocity);
    SmartDashboard.putBoolean("shooter/inTrench", inTrench);
  }

  private double getHubPositionX() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubX;
    }
    return FieldConstants.blueHubX;
  }

  private double getHubPositionY() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubY;
    }
    return FieldConstants.blueHubY;
  }

  private Translation2d getHubPosition() {
    return new Translation2d(getHubPositionX(), getHubPositionY());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    angleSubsystem.lockCurrentAngle();
  }
}
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  /* ===================== CHANGE THESE ===================== */
  public static final double FLYWHEEL_SPEED = -1.0; // 0..1
  public static final double FEEDER_SPEED   = 1.00; // 0..1
   public static final double FEEDER_SPEED_SLOW   = 0.50; // 0..1
    public static final double FEEDER_SPEED_SLOWER   = 0.25; // 0..1

  public static final boolean FOLLOWER2_INVERT = false;
  public static final boolean FOLLOWER3_INVERT = true;

  // If the second feeder motor spins backwards, change this to true
  public static final boolean FEEDER_FOLLOWER_INVERT = true;

  public static final double MOTOR_TO_WHEEL_RATIO = 2.0;
  /* ======================================================== */

  private final SparkMax flywheelLeader;
  private final SparkMax flywheelFollower2;
  private final SparkMax flywheelFollower3;

  private final SparkMax feederLeader;
  private final SparkMax feederFollower;

  private final RelativeEncoder flywheelEncoder;

  public ShooterSubsystem(
      int leaderId,
      int follower2Id,
      int follower3Id,
      int feederLeaderId,
      int feederFollowerId
  ) {
    flywheelLeader = new SparkMax(leaderId, MotorType.kBrushless);
    flywheelFollower2 = new SparkMax(follower2Id, MotorType.kBrushless);
    flywheelFollower3 = new SparkMax(follower3Id, MotorType.kBrushless);

    feederLeader = new SparkMax(feederLeaderId, MotorType.kBrushless);
    feederFollower = new SparkMax(feederFollowerId, MotorType.kBrushless);

    flywheelEncoder = flywheelLeader.getEncoder();

    // ----- Flywheel leader config -----
    SparkMaxConfig leaderCfg = new SparkMaxConfig();
    leaderCfg.idleMode(IdleMode.kCoast);
    flywheelLeader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ----- Flywheel follower 2 config -----
    SparkMaxConfig followerCfg2 = new SparkMaxConfig();
    followerCfg2.apply(leaderCfg);
    followerCfg2.follow(leaderId, FOLLOWER2_INVERT);
    flywheelFollower2.configure(followerCfg2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ----- Flywheel follower 3 config -----
    SparkMaxConfig followerCfg3 = new SparkMaxConfig();
    followerCfg3.apply(leaderCfg);
    followerCfg3.follow(leaderId, FOLLOWER3_INVERT);
    flywheelFollower3.configure(followerCfg3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ----- Feeder leader config -----
    SparkMaxConfig feederLeaderCfg = new SparkMaxConfig();
    feederLeaderCfg.idleMode(IdleMode.kBrake);
    feederLeader.configure(feederLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ----- Feeder follower config -----
    SparkMaxConfig feederFollowerCfg = new SparkMaxConfig();
    feederFollowerCfg.apply(feederLeaderCfg);
    feederFollowerCfg.follow(feederLeaderId, FEEDER_FOLLOWER_INVERT);
    feederFollower.configure(feederFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* -------- Flywheels -------- */
  public void flywheelsOn() {
    flywheelLeader.set(MathUtil.clamp(FLYWHEEL_SPEED, -1.0, 1.0));
  }

  public void flywheelsOff() {
    flywheelLeader.set(0.0);
  }

  public void setFlywheelSpeed(double speed) {
    flywheelLeader.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  /* -------- Feeders -------- */
  public void feederOn() {
    feederLeader.set(MathUtil.clamp(FEEDER_SPEED, -1.0, 1.0));
  }

  public void feederOff() {
    feederLeader.set(0.0);
  }

  public void setFeederSpeed(double speed) {
    feederLeader.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  /* -------- RPM Measurement -------- */
  public double getMotorRPM() {
    return flywheelEncoder.getVelocity();
  }

  public double getWheelRPM() {
    return getMotorRPM() * MOTOR_TO_WHEEL_RATIO;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Motor RPM", getMotorRPM());
    SmartDashboard.putNumber("Shooter Wheel RPM", getWheelRPM());
  }
}
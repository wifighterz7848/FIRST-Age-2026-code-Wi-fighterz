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

  /* ===================== CHANGE / TUNE THESE ===================== */
  // Feeder
  public static final double FEEDER_SPEED = 1.00; // 0..1
  public static final boolean FEEDER_FOLLOWER_INVERT = true;

  // Flywheel followers
  public static final boolean FOLLOWER2_INVERT = false;
  public static final boolean FOLLOWER3_INVERT = true;

  // Shooter forward sign (flip if direction is wrong)
  private static final double FORWARD_SIGN = -1.0;

  // POV power step mode
  private static final double POWER_STEP = 0.025; // 2.5% per click
  private static final double MAX_POWER = 1.0;    // 0..1 magnitude
  /* ================================================================ */

  private final SparkMax flywheelLeader;
  private final SparkMax flywheelFollower2;
  private final SparkMax flywheelFollower3;

  private final SparkMax feederLeader;
  private final SparkMax feederFollower;

  private final RelativeEncoder flywheelEncoder; // motor encoder (for RPM display)

  // Flywheel power state (0..1 magnitude)
  private double flywheelPower = 0.0;

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

    // Flywheel leader config
    SparkMaxConfig leaderCfg = new SparkMaxConfig();
    leaderCfg.idleMode(IdleMode.kCoast);
    flywheelLeader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Flywheel followers
    SparkMaxConfig followerCfg2 = new SparkMaxConfig();
    followerCfg2.apply(leaderCfg);
    followerCfg2.follow(leaderId, FOLLOWER2_INVERT);
    flywheelFollower2.configure(followerCfg2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followerCfg3 = new SparkMaxConfig();
    followerCfg3.apply(leaderCfg);
    followerCfg3.follow(leaderId, FOLLOWER3_INVERT);
    flywheelFollower3.configure(followerCfg3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Feeder leader config
    SparkMaxConfig feederLeaderCfg = new SparkMaxConfig();
    feederLeaderCfg.idleMode(IdleMode.kBrake);
    feederLeader.configure(feederLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Feeder follower config
    SparkMaxConfig feederFollowerCfg = new SparkMaxConfig();
    feederFollowerCfg.apply(feederLeaderCfg);
    feederFollowerCfg.follow(feederLeaderId, FEEDER_FOLLOWER_INVERT);
    feederFollower.configure(feederFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Safety
    feederLeader.set(0.0);
    flywheelLeader.set(0.0);
  }

  /* ---------------- Flywheel power control ---------------- */

  /** Set flywheel power 0..1 magnitude (applies FORWARD_SIGN). */
  public void setFlywheelPower(double power01) {
    flywheelPower = MathUtil.clamp(power01, 0.0, MAX_POWER);
    flywheelLeader.set(FORWARD_SIGN * flywheelPower);
  }

  public void stepPowerUp() {
    setFlywheelPower(flywheelPower + POWER_STEP);
  }

  public void stepPowerDown() {
    setFlywheelPower(flywheelPower - POWER_STEP);
  }

  public void stopFlywheelPower() {
    setFlywheelPower(0.0);
  }

  public double getFlywheelPower() {
    return flywheelPower;
  }

  /* ---------------- Feeder ---------------- */

  public void feederOn() {
    feederLeader.set(MathUtil.clamp(FEEDER_SPEED, -1.0, 1.0));
  }

  public void feederOff() {
    feederLeader.set(0.0);
  }

  /* ---------------- Debug RPM (motor encoder) ---------------- */

  public double getMotorRPM() {
    return flywheelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Power", flywheelPower);
    SmartDashboard.putNumber("Flywheel Motor RPM", getMotorRPM() * 2);
  }
}

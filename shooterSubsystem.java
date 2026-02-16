package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {

  // ====== Change these later ======
  public static final double SHOOTER_SPEED = 0.90; // 0..1
  public static final double ADJUST_SPEED  = 0.20; // slow
  // ================================

  private final SparkMax shooter1;   // leader
  private final SparkMax shooter2;   // follower
  private final SparkMax shooter3;   // follower

  private final SparkMax adjustMotor;

  public shooterSubsystem(int shooter1Id, int shooter2Id, int shooter3Id, int adjustId) {
    shooter1 = new SparkMax(shooter1Id, MotorType.kBrushless);
    shooter2 = new SparkMax(shooter2Id, MotorType.kBrushless);
    shooter3 = new SparkMax(shooter3Id, MotorType.kBrushless);

    adjustMotor = new SparkMax(adjustId, MotorType.kBrushless);

    // -------- Shooter config (leader) --------
    SparkMaxConfig shooterLeaderCfg = new SparkMaxConfig();
    shooterLeaderCfg.idleMode(IdleMode.kCoast);
    shooterLeaderCfg.smartCurrentLimit(40);

    shooter1.configure(shooterLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------- Shooter config (followers) --------
    // Copy leader settings, then set follower mode
    SparkMaxConfig shooterFollowerCfg = new SparkMaxConfig();
    shooterFollowerCfg.apply(shooterLeaderCfg);

    // If a follower needs opposite direction, change false -> true
    shooterFollowerCfg.follow(shooter1Id, false);

    shooter2.configure(shooterFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooter3.configure(shooterFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------- Adjust config --------
    SparkMaxConfig adjustCfg = new SparkMaxConfig();
    adjustCfg.idleMode(IdleMode.kCoast);
    adjustCfg.smartCurrentLimit(25);

    adjustMotor.configure(adjustCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---- Shooter ----
  public void shooterOn() {
    double s = MathUtil.clamp(SHOOTER_SPEED, -1.0, 1.0);
    shooter1.set(s);   // only set leader; followers mirror automatically
  }

  public void shooterOff() {
    shooter1.set(0.0);
  }

  // ---- Adjust (D-pad) ----
  public void adjustUp() {
    adjustMotor.set(MathUtil.clamp(+ADJUST_SPEED, -1.0, 1.0));
  }

  public void adjustDown() {
    adjustMotor.set(MathUtil.clamp(-ADJUST_SPEED, -1.0, 1.0));
  }

  public void adjustStop() {
    adjustMotor.set(0.0);
  }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {

  // ====== Change these later ======
  public static final double SHOOTER_SPEED = 0.90; // 0..1
  public static final double ADJUST_SPEED  = 0.20; // slow
  // ================================

  private final SparkMax shooter1;
  private final SparkMax shooter2;
  private final SparkMax shooter3;

  private final SparkMax adjustMotor;

  public shooterSubsystem(int shooter1Id, int shooter2Id, int shooter3Id, int adjustId) {
    shooter1 = new SparkMax(shooter1Id, MotorType.kBrushless);
    shooter2 = new SparkMax(shooter2Id, MotorType.kBrushless);
    shooter3 = new SparkMax(shooter3Id, MotorType.kBrushless);

    adjustMotor = new SparkMax(adjustId, MotorType.kBrushless);

    // Shooter config (usually coast for flywheels)
    SparkMaxConfig shooterCfg = new SparkMaxConfig();
    shooterCfg.idleMode(IdleMode.kCoast);
    shooterCfg.smartCurrentLimit(40);

    shooter1.configure(shooterCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooter2.configure(shooterCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooter3.configure(shooterCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Adjust config (usually brake so it holds position better)
    SparkMaxConfig adjustCfg = new SparkMaxConfig();
    adjustCfg.idleMode(IdleMode.kBrake);
    adjustCfg.smartCurrentLimit(25);

    adjustMotor.configure(adjustCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---- Shooter ----
  public void shooterOn() {
    double s = MathUtil.clamp(SHOOTER_SPEED, -1.0, 1.0);
    shooter1.set(s);
    shooter2.set(s);
    shooter3.set(s);
  }

  public void shooterOff() {
    shooter1.set(0);
    shooter2.set(0);
    shooter3.set(0);
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

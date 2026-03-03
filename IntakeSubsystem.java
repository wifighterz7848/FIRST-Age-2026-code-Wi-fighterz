package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  /* ===================== CHANGE THESE ===================== */
  public static final int ARM_LEADER_ID   = 2;
  public static final int ARM_FOLLOWER_ID = 1;
  public static final int SPINNER_ID      = 3;

  public static final double ARM_SPEED     = 0.10;
  public static final double SPINNER_SPEED = 0.65;
  /* ======================================================== */

  private final SparkMax armLeader;
  private final SparkMax armFollower;
  private final SparkMax spinnerMotor;

  public IntakeSubsystem() {
    armLeader = new SparkMax(ARM_LEADER_ID, MotorType.kBrushless);
    armFollower = new SparkMax(ARM_FOLLOWER_ID, MotorType.kBrushless);
    spinnerMotor = new SparkMax(SPINNER_ID, MotorType.kBrushless);

    // -------- Arm leader config --------
    SparkMaxConfig armLeaderCfg = new SparkMaxConfig();
    armLeaderCfg.idleMode(IdleMode.kBrake);
    armLeaderCfg.smartCurrentLimit(30);
    armLeader.configure(armLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------- Arm follower config --------
    SparkMaxConfig armFollowerCfg = new SparkMaxConfig();
    armFollowerCfg.apply(armLeaderCfg);
    armFollowerCfg.follow(ARM_LEADER_ID, true);
    armFollower.configure(armFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------- Spinner config --------
    SparkMaxConfig spinCfg = new SparkMaxConfig();
    spinnerMotor.configure(spinCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* ---------------- Arm controls ---------------- */
  public void armUp() {
    armLeader.set(MathUtil.clamp(+ARM_SPEED, -1.0, 1.0));
  }

  public void armDown() {
    armLeader.set(MathUtil.clamp(-ARM_SPEED, -1.0, 1.0));
  }

  public void stopArm() {
    armLeader.set(0.0);
  }

  /* ---------------- Intake / spinner controls ---------------- */
  public void spinOn() {
    spinnerMotor.set(MathUtil.clamp(+SPINNER_SPEED, -1.0, 1.0));
  }

  public void spinReverse() {
    spinnerMotor.set(MathUtil.clamp(-SPINNER_SPEED, -1.0, 1.0));
  }

  public void spinOff() {
    spinnerMotor.set(0.0);
  }

  public void setSpinnerSpeed(double speed) {
    spinnerMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }
}
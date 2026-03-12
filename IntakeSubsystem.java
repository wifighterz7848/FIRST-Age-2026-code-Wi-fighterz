package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // ====== IDs ======
  public static final int ARM_MOTOR_A_ID = 2;   // formerly leader
  public static final int ARM_MOTOR_B_ID = 1;   // formerly follower
  public static final int SPINNER_ID     = 3;

  // ====== Spinner ======
  public static final double SPINNER_SPEED = 0.65;

  // ====== Position control tuning ======
  private static final double KP_A = 0.18;      // tune
  private static final double KP_B = 0.18;      // tune
  private static final double TOL_ROT = 0.25;   // rotations tolerance for each motor

  // defaults if you don't pass max speeds
  private static final double MAX_A_DEFAULT = 0.75;
  private static final double MAX_B_DEFAULT = 0.75;

  private final SparkMax armA;
  private final SparkMax armB;
  private final SparkMax spinnerMotor;

  private final RelativeEncoder encA;
  private final RelativeEncoder encB;

  private boolean posEnabled = false;
  private double targetA = 5.0;
  private double targetB = 9.0;
  private double maxA = MAX_A_DEFAULT;
  private double maxB = MAX_B_DEFAULT;

  public IntakeSubsystem() {
    armA = new SparkMax(ARM_MOTOR_A_ID, MotorType.kBrushless);
    armB = new SparkMax(ARM_MOTOR_B_ID, MotorType.kBrushless);
    spinnerMotor = new SparkMax(SPINNER_ID, MotorType.kBrushless);

    SparkMaxConfig armCfg = new SparkMaxConfig();
    armCfg.idleMode(IdleMode.kBrake);
    armCfg.smartCurrentLimit(30);
    armA.configure(armCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armB.configure(armCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig spinCfg = new SparkMaxConfig();
    spinCfg.idleMode(IdleMode.kCoast);
    spinCfg.smartCurrentLimit(40);
    spinnerMotor.configure(spinCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encA = armA.getEncoder();
    encB = armB.getEncoder();

    encA.setPosition(0.0);
    encB.setPosition(0.0);

    stopArm();
    spinOff();
  }

  /* ---------------- Dual-motor arm API ---------------- */

  public void zeroArmEncoders() {
    encA.setPosition(0.0);
    encB.setPosition(0.0);
    targetA = 0.0;
    targetB = 0.0;
  }

  public double getRotA() { return encA.getPosition(); }
  public double getRotB() { return encB.getPosition(); }

  /** Move both motors by different rotation deltas using one call. */
  public void moveArmByRotations(double deltaA, double deltaB) {
    moveArmByRotations(deltaA, deltaB, MAX_A_DEFAULT, MAX_B_DEFAULT);
  }

  /** Move both motors by different rotation deltas AND different max speeds. */
  public void moveArmByRotations(double deltaA, double deltaB, double maxOutA, double maxOutB) {
    targetA = getRotA() + deltaA;
    targetB = getRotB() + deltaB;

    maxA = MathUtil.clamp(Math.abs(maxOutA), 0.0, 1.0);
    maxB = MathUtil.clamp(Math.abs(maxOutB), 0.0, 1.0);

    posEnabled = true;
  }

  public boolean atTargets() {
    return Math.abs(targetA - getRotA()) <= TOL_ROT
        && Math.abs(targetB - getRotB()) <= TOL_ROT;
  }

  public void stopArm() {
    posEnabled = false;
    armA.set(0.0);
    armB.set(0.0);
  }

  /* ---------------- Spinner ---------------- */

  public void spinOn() {
    spinnerMotor.set(MathUtil.clamp(+SPINNER_SPEED, -1.0, 1.0));
  }

  public void spinReverse() {
    spinnerMotor.set(MathUtil.clamp(-SPINNER_SPEED, -1.0, 1.0));
  }

  public void spinOff() {
    spinnerMotor.set(0.0);
  }

  /* ---------------- Periodic control ---------------- */

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // don’t keep moving when disabled
      posEnabled = false;
      armA.set(0.0);
      armB.set(0.0);
    } else if (posEnabled) {
      double errA = targetA - getRotA();
      double errB = targetB - getRotB();

      boolean doneA = Math.abs(errA) <= TOL_ROT;
      boolean doneB = Math.abs(errB) <= TOL_ROT;

      if (doneA) armA.set(0.0);
      else {
        double outA = MathUtil.clamp(errA * KP_A, -maxA, maxA);
        armA.set(outA);
      }

      if (doneB) armB.set(0.0);
      else {
        double outB = MathUtil.clamp(errB * KP_B, -maxB, maxB);
        armB.set(outB);
      }

      if (doneA && doneB) posEnabled = false;
    }

    SmartDashboard.putNumber("ArmA Rot", getRotA());
    SmartDashboard.putNumber("ArmB Rot", getRotB());
    SmartDashboard.putNumber("ArmA Target", targetA);
    SmartDashboard.putNumber("ArmB Target", targetB);
    SmartDashboard.putBoolean("Arm Dual Pos Enabled", posEnabled);
  }
}

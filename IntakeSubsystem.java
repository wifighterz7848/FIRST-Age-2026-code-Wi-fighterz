package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // ================= Arm =================
  public static final double ARM_SPEED = 0.20;

  public static final double BOTTOM_DEG = 358.0;
  public static final double TOP_DEG = 285.0;

  private static final double TOLERANCE_DEG = 2.0;
  private static final boolean ENCODER_INVERTED = false;

  private final SparkMax armMotor;
  private final AbsoluteEncoder absEncoder;

  private double targetDeg = BOTTOM_DEG;
  private boolean seeking = false;

  // ================= Spinner (CAN 12) =================
  // CHANGE THIS ONE NUMBER for intake spinner speed
  public static final double SPINNER_SPEED = -0.6;

  private final SparkMax spinnerMotor;

  public IntakeSubsystem(int armCanId) {
    armMotor = new SparkMax(armCanId, MotorType.kBrushless);
    absEncoder = armMotor.getAbsoluteEncoder();

    // Arm config
    SparkMaxConfig armCfg = new SparkMaxConfig();
    armCfg.idleMode(IdleMode.kBrake);
    armCfg.smartCurrentLimit(35);
    armMotor.configure(armCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Spinner motor on CAN ID 12
    spinnerMotor = new SparkMax(12, MotorType.kBrushless);

    SparkMaxConfig spinCfg = new SparkMaxConfig();
    spinCfg.idleMode(IdleMode.kCoast);      // usually better for rollers/flywheels
    spinCfg.smartCurrentLimit(40);
    spinnerMotor.configure(spinCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Arm Deg", 0.0);
    SmartDashboard.putNumber("Arm Deg Normalized", 0.0);
    SmartDashboard.putNumber("Arm Target Deg", targetDeg);
    SmartDashboard.putNumber("Arm Error", 0.0);
    SmartDashboard.putBoolean("Arm Seeking", false);
  }

  // ---------- Spinner controls ----------
  public void spinOn() {
    spinnerMotor.set(MathUtil.clamp(SPINNER_SPEED, -1.0, 1.0));
  }

  public void spinOff() {
    spinnerMotor.set(0.0);
  }

  // ---------- Arm controls ----------
  public void goBottom() {
    targetDeg = BOTTOM_DEG;
    seeking = true;
    SmartDashboard.putNumber("Arm Target Deg", targetDeg);
    SmartDashboard.putBoolean("Arm Seeking", true);
  }

  public void goTop() {
    targetDeg = TOP_DEG;
    seeking = true;
    SmartDashboard.putNumber("Arm Target Deg", targetDeg);
    SmartDashboard.putBoolean("Arm Seeking", true);
  }

  public void up() {
    seeking = false;
    SmartDashboard.putBoolean("Arm Seeking", false);
    armMotor.set(clampSpeed(+ARM_SPEED));
  }

  public void down() {
    seeking = false;
    SmartDashboard.putBoolean("Arm Seeking", false);
    armMotor.set(clampSpeed(-ARM_SPEED));
  }

  public void stopArm() {
    seeking = false;
    SmartDashboard.putBoolean("Arm Seeking", false);
    armMotor.set(0.0);
  }

  public double getAbsRot() {
    return absEncoder.getPosition();
  }

  public double getAbsDegRaw() {
    return getAbsRot() * 360.0;
  }

  public double getArmDeg() {
    double deg = absEncoder.getPosition() * 360.0;
    if (ENCODER_INVERTED) deg = -deg;

    // normalize to [0, 360)
    deg = ((deg % 360.0) + 360.0) % 360.0;
    return deg;
  }

  private double clampSpeed(double s) {
    return MathUtil.clamp(s, -ARM_SPEED, ARM_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Rot", getAbsRot());
    SmartDashboard.putNumber("Arm Encoder Deg Raw", getAbsDegRaw());

    double deg = getArmDeg();
    SmartDashboard.putNumber("Arm Deg", deg);
    SmartDashboard.putNumber("Arm Deg Normalized", deg);

    if (!seeking) return;

    // wrap-safe error on a circle
    double error = MathUtil.inputModulus(targetDeg - deg, -180.0, 180.0);
    SmartDashboard.putNumber("Arm Error", error);

    if (Math.abs(error) <= TOLERANCE_DEG) {
      armMotor.set(0.0);
      seeking = false;
      SmartDashboard.putBoolean("Arm Seeking", false);
      return;
    }

    if (error > 0) {
      armMotor.set(clampSpeed(+ARM_SPEED));
    } else {
      armMotor.set(clampSpeed(-ARM_SPEED));
    }
  }
}

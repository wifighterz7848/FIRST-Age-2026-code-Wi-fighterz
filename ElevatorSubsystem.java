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

public class ElevatorSubsystem extends SubsystemBase {

  // ====== CHANGE THESE ======
  private static final int ELEVATOR_MOTOR_ID = 32;
  private static final boolean INVERT_MOTOR = false;
  private static final int CURRENT_LIMIT = 40;

  // Position control tuning
  private static final double POS_KP = 0.18;      // tune
  private static final double POS_TOL = 0.15;     // rotations
  private static final double MAX_AUTO_UP = 0.5;  // cap up output
  private static final double MAX_AUTO_DOWN = 0.4; // cap down output
  // ==========================

  private final SparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;

  private boolean positionControlEnabled = false;
  private double targetRotations = 0.0;

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kBrake);
    cfg.smartCurrentLimit(CURRENT_LIMIT);
    cfg.inverted(INVERT_MOTOR);
    elevatorMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPosition(0.0);

    stop();
  }

  public void stop() {
    positionControlEnabled = false;
    elevatorMotor.set(0.0);
  }

  public double getRotations() {
    return elevatorEncoder.getPosition();
  }

  public void zeroEncoder() {
    elevatorEncoder.setPosition(0.0);
    targetRotations = 0.0;
  }

  public void moveByRotations(double deltaRotations) {
    targetRotations = getRotations() + deltaRotations;
    positionControlEnabled = true;
  }

  public boolean atTarget() {
    return Math.abs(targetRotations - getRotations()) <= POS_TOL;
  }

  @Override
  public void periodic() {
    if (positionControlEnabled) {
      double error = targetRotations - getRotations();

      if (Math.abs(error) <= POS_TOL) {
        elevatorMotor.set(0.0);
        positionControlEnabled = false;
      } else {
        double output = error * POS_KP;

        if (output > 0) output = MathUtil.clamp(output, 0.0, MAX_AUTO_UP);
        else           output = MathUtil.clamp(output, -MAX_AUTO_DOWN, 0.0);

        elevatorMotor.set(output);
      }
    }

    SmartDashboard.putNumber("Elevator Rotations", getRotations());
    SmartDashboard.putNumber("Elevator Target Rotations", targetRotations);
    SmartDashboard.putBoolean("Elevator At Target", atTarget());
    SmartDashboard.putBoolean("Elevator Position Control", positionControlEnabled);
  }
}

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class RobotContainer {

  /* ---------------- PathPlanner Auto Chooser ---------------- */
  private final SendableChooser<Command> autoChooser;
  private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

  /* ---------------- Auto-aim tuning ---------------- */
  private static final String LIMELIGHT_LEFT = "limelight-left";
  private static final String LIMELIGHT_RIGHT = "limelight-right";

  private static final double LEFT_CAM_YAW_OFFSET  = -30.0;
  private static final double RIGHT_CAM_YAW_OFFSET =  30.0;

  private static final double AIM_KP = 0.02;
  private static final double AIM_DEADBAND = 0.2;

  /* ---------------- Subsystems ---------------- */
private final DriveSubsystem drive;
private final IntakeSubsystem arm = new IntakeSubsystem();
private final ShooterSubsystem shooter = new ShooterSubsystem(21, 22, 23, 30, 31);




  /* ---------------- Controllers ---------------- */
  private final CommandXboxController driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // IMPORTANT: make sure this is a DIFFERENT USB port than the driver controller
  private final CommandXboxController shooterController =
      new CommandXboxController(1);

  /* ---------------- Drive State ---------------- */
  private double swerveSpeedScaleTranslation = 0.1;
  private double swerveSpeedScaleRotation = 0.25;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);

    drive = DriveSubsystem.getInstance();

   registerAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);

    configureButtonBindings();
  }

  /* ---------------- Speed Modes ---------------- */

  private void setNormalSpeed() {
    swerveSpeedScaleTranslation = 0.35;
    swerveSpeedScaleRotation = 0.35;
  }

   private void setTurboSpeed() {
    swerveSpeedScaleTranslation = 1.00;
    swerveSpeedScaleRotation = 1.00;
  }

  /* ---------------- Teleop ---------------- */
public void teleopInit() {
  setNormalSpeed();
  drive.resetFieldOrientedDir();
}

  /* ---------------- Joystick Processing ---------------- */
  private double cookRawJoystickInput(double v, double scale) {
    double vr = MathUtil.applyDeadband(v, OIConstants.kDriveDeadband);
    vr = vr * vr * vr;
    return MathUtil.clamp(vr, -1.0, 1.0) * scale;
  }

  /* ---------------- Button Bindings ---------------- */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
      new RunCommand(() -> {
        updateLimelightDashboard();

        double ybump = 0.0;
        if (driverController.x().getAsBoolean()) ybump += 0.025;
        if (driverController.b().getAsBoolean()) ybump -= 0.025;

        double rotation;
        boolean autoAimEnabled = driverController.leftTrigger(0.5).getAsBoolean();
        SmartDashboard.putBoolean("AutoAim Enabled", autoAimEnabled);

        if (autoAimEnabled) {
          rotation = getAutoAimRotation();
        } else {
           double shooterRotate = cookRawJoystickInput(
        shooterController.getHID().getRawAxis(0),   // shooter left stick X
        -swerveSpeedScaleRotation
    );

    double driverRotate = cookRawJoystickInput(
        driverController.getHID().getRawAxis(4),    // driver right stick X
        -swerveSpeedScaleRotation
    );

    // Shooter controller has priority.
    // If shooter is not rotating, fall back to driver controller.
    if (Math.abs(shooterRotate) > 0.01) {
        rotation = shooterRotate;
    } else {
        rotation = driverRotate;
    }
}

        drive.drive(
            cookRawJoystickInput(driverController.getHID().getRawAxis(1), -swerveSpeedScaleTranslation),
            cookRawJoystickInput(driverController.getHID().getRawAxis(0), -swerveSpeedScaleTranslation),
            rotation,
            0,
            ybump
        );

      }, drive)
    );

    driverController.leftTrigger(0.5)
      .whileTrue(new RunCommand(drive::setX, drive));

    driverController.rightStick()
      .onTrue(new InstantCommand(drive::resetFieldOrientedDir, drive));

// Arm up while held
shooterController.x()
  .whileTrue(new RunCommand(arm::armUp, arm))
  .onFalse(new InstantCommand(arm::stopArm, arm));

// Arm down while held
shooterController.y()
  .whileTrue(new RunCommand(arm::armDown, arm))
  .onFalse(new InstantCommand(arm::stopArm, arm));

// Intake forward while held
driverController.rightBumper()
  .whileTrue(new RunCommand(arm::spinOn, arm))
  .onFalse(new InstantCommand(arm::spinOff, arm));

// Intake reverse while held
driverController.leftBumper()
  .whileTrue(new RunCommand(arm::spinReverse, arm))
  .onFalse(new InstantCommand(arm::spinOff, arm));

driverController.rightTrigger(0.5)
  .onTrue(new InstantCommand(this::setTurboSpeed))
  .onFalse(new InstantCommand(this::setNormalSpeed));

shooterController.rightTrigger(0.05)
  .whileTrue(new RunCommand(() -> {
    double trigger = shooterController.getRightTriggerAxis(); // 0.0 to 1.0
    double shooterSpeed = -0.6 * trigger;                  // maps trigger % to motor %
    shooter.setFlywheelSpeed(shooterSpeed);
  }, shooter))
  .onFalse(new InstantCommand(shooter::flywheelsOff, shooter));

shooterController.leftTrigger(0.5)
  .whileTrue(new RunCommand(shooter::feederOn, shooter))
  .onFalse(new InstantCommand(shooter::feederOff, shooter));




  }

  /* ---------------- Autonomous ---------------- */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void registerAutoCommands() {
  // Intake (spinner) for picking up
  NamedCommands.registerCommand("IntakeOn",
      Commands.startEnd(arm::spinOn, arm::spinOff, arm));

  NamedCommands.registerCommand("IntakeReverse",
      Commands.startEnd(arm::spinReverse, arm::spinOff, arm));

  // Feeder (push note into shooter)
  NamedCommands.registerCommand("FeederOn",
      Commands.startEnd(shooter::feederOn, shooter::feederOff, shooter));

      NamedCommands.registerCommand("FlywheelsOn",
    Commands.startEnd(() -> shooter.setFlywheelSpeed(-0.5), shooter::flywheelsOff, shooter));

    NamedCommands.registerCommand("AutoShoot",
  Commands.sequence(
      Commands.runOnce(() -> shooter.setFlywheelSpeed(-0.338), shooter),
      Commands.waitSeconds(3), // Spin up flywheels for 3 seconds
      Commands.startEnd(shooter::feederOn, shooter::feederOff, shooter).withTimeout(3)
  ).finallyDo(interrupted -> {
      shooter.feederOff();
      shooter.flywheelsOff();
  })
);
}

  /* ---------------- Limelight Dashboard ---------------- */
  private void updateLimelightDashboard() {
    SmartDashboard.putBoolean("LL Left Has Target", LimelightHelpers.getTV(LIMELIGHT_LEFT));
    SmartDashboard.putNumber("LL Left TX Raw", LimelightHelpers.getTX(LIMELIGHT_LEFT));

    SmartDashboard.putBoolean("LL Right Has Target", LimelightHelpers.getTV(LIMELIGHT_RIGHT));
    SmartDashboard.putNumber("LL Right TX Raw", LimelightHelpers.getTX(LIMELIGHT_RIGHT));

    double midLeft = getMidFromCamera(LIMELIGHT_LEFT, LEFT_CAM_YAW_OFFSET);
    double midRight = getMidFromCamera(LIMELIGHT_RIGHT, RIGHT_CAM_YAW_OFFSET);

    SmartDashboard.putNumber("LL Mid Left (Robot)", midLeft);
    SmartDashboard.putNumber("LL Mid Right (Robot)", midRight);
  }

  /* ---------------- Auto Aim ---------------- */
  private double getAutoAimRotation() {
    double txLeft = getMidFromCamera(LIMELIGHT_LEFT, LEFT_CAM_YAW_OFFSET);
    double txRight = getMidFromCamera(LIMELIGHT_RIGHT, RIGHT_CAM_YAW_OFFSET);

    boolean leftValid = !Double.isNaN(txLeft);
    boolean rightValid = !Double.isNaN(txRight);

    if (!leftValid && !rightValid) return 0.0;

    double txMid = leftValid && rightValid ? (txLeft + txRight) / 2.0 : (leftValid ? txLeft : txRight);

    if (Math.abs(txMid) < AIM_DEADBAND) return 0.0;

    return MathUtil.clamp(-txMid * AIM_KP, -0.25, 0.25);
  }

  private double getMidFromCamera(String name, double yawOffset) {
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
    if (fiducials == null || fiducials.length == 0) return Double.NaN;

    boolean found9 = false, found10 = false;
    double tx9 = 0.0, tx10 = 0.0;

    for (LimelightHelpers.RawFiducial f : fiducials) {
      if (f.id == 9) { tx9 = f.txnc + yawOffset; found9 = true; }
      else if (f.id == 10) { tx10 = f.txnc + yawOffset; found10 = true; }
    }

    if (found9 && found10) return (tx9 + tx10) / 2.0;
    if (found9) return tx9;
    if (found10) return tx10;
    return Double.NaN;
  }


}

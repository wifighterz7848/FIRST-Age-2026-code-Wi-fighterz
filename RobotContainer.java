// RobotContainer.java (WORKING “JITTERY” VERSION)
// - Aims using pose.getRotation() (matches your version that pointed correctly)
// - Auto aim held on DRIVER LEFT BUMPER
// - No getAbsoluteHeading(), no getFieldHeading() required for aiming
package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

  private final DriveSubsystem drive;
  private final IntakeSubsystem arm = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(21, 22, 23, 30, 31);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final CommandXboxController driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController shooterController =
      new CommandXboxController(OIConstants.kShooterControllerPort);

  private double swerveSpeedScaleTranslation = 0.1;
  private double swerveSpeedScaleRotation = 0.25;

  private final PIDController hubAimPID = new PIDController(4.0, 0.0, 0.2);
  private static final double HUB_AIM_MAX_ROT_CMD = 0.60;
  private static final boolean INVERT_AIM_OUTPUT = false;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);

    drive = DriveSubsystem.getInstance();

    hubAimPID.enableContinuousInput(-Math.PI, Math.PI);

    registerAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);

    configureButtonBindings();
  }

  private void setNormalSpeed() {
    swerveSpeedScaleTranslation = 0.35;
    swerveSpeedScaleRotation = 0.35;
  }

  private void setTurboSpeed() {
    swerveSpeedScaleTranslation = 1.00;
    swerveSpeedScaleRotation = 1.00;
  }

  public void teleopInit() {
    setNormalSpeed();
    drive.resetFieldOrientedDir();
  }

  private double cookRawJoystickInput(double v, double scale) {
    double vr = MathUtil.applyDeadband(v, OIConstants.kDriveDeadband);
    vr = vr * vr * vr;
    return MathUtil.clamp(vr, -1.0, 1.0) * scale;
  }

  // WORKING “JITTERY” AIM: uses pose.getRotation() directly
  private double getHubAimRotationCmd() {
    Pose2d pose = drive.getPose();
    Translation2d robot = pose.getTranslation();

    boolean isRed = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Translation2d hub = isRed ? VisionShootConstants.RED_HUB_CENTER
                              : VisionShootConstants.BLUE_HUB_CENTER;

    Translation2d toHub = hub.minus(robot);
    Rotation2d desired = new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));

    double omega = hubAimPID.calculate(pose.getRotation().getRadians(), desired.getRadians());
    omega = MathUtil.clamp(omega, -HUB_AIM_MAX_ROT_CMD, HUB_AIM_MAX_ROT_CMD);

    if (INVERT_AIM_OUTPUT) omega = -omega;
    return omega;
  }

  private void configureButtonBindings() {

    drive.setDefaultCommand(
      new RunCommand(() -> {

        double ybump = 0.0;
        if (driverController.x().getAsBoolean()) ybump += 0.025;
        if (driverController.b().getAsBoolean()) ybump -= 0.025;

        double shooterRotate = cookRawJoystickInput(
            shooterController.getHID().getRawAxis(0),
            -swerveSpeedScaleRotation
        );

        double driverRotate = cookRawJoystickInput(
            driverController.getHID().getRawAxis(4),
            -swerveSpeedScaleRotation
        );

        double rotation;
        if (driverController.leftBumper().getAsBoolean()) {
          rotation = getHubAimRotationCmd();  // AUTO AIM
        } else {
          rotation = (Math.abs(shooterRotate) > 0.01) ? shooterRotate : driverRotate;
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

    shooterController.x().onTrue(
      new InstantCommand(() -> arm.moveArmByRotations(9.0, -16.2, 0.56, 1.0), arm)
    );

    shooterController.b().onTrue(new InstantCommand(() -> elevator.moveByRotations(185.0), elevator));
    shooterController.a().onTrue(new InstantCommand(() -> elevator.moveByRotations(-185.0), elevator));
    shooterController.povDown().onTrue(new InstantCommand(() -> elevator.moveByRotations(-1.0), elevator));
    shooterController.povUp().onTrue(new InstantCommand(() -> elevator.moveByRotations(1.0), elevator));

    driverController.rightBumper()
      .whileTrue(new RunCommand(arm::spinOn, arm))
      .onFalse(new InstantCommand(arm::spinOff, arm));

    driverController.leftBumper()
      .whileTrue(new RunCommand(arm::spinReverse, arm))
      .onFalse(new InstantCommand(arm::spinOff, arm));

    driverController.rightTrigger(0.5)
      .onTrue(new InstantCommand(this::setTurboSpeed))
      .onFalse(new InstantCommand(this::setNormalSpeed));

    final double DELTA_A = 3.61;
    final double DELTA_B = -7.0;
    final double MAX_A = 0.40;
    final double MAX_B = 1.0;
    final double BUMP_WAIT = 0.55;

    shooterController.leftTrigger(0.5).whileTrue(
      Commands.startEnd(shooter::feederOn, shooter::feederOff, shooter)
        .alongWith(
          Commands.sequence(
            Commands.runOnce(() -> arm.moveArmByRotations(-DELTA_A, -DELTA_B, MAX_A, MAX_B), arm),
            Commands.waitSeconds(BUMP_WAIT),
            Commands.runOnce(() -> arm.moveArmByRotations(+DELTA_A, +DELTA_B, MAX_A, MAX_B), arm),
            Commands.waitSeconds(BUMP_WAIT)
          ).repeatedly()
        )
        .finallyDo(i -> arm.stopArm())
    );

    shooterController.povRight().onTrue(new InstantCommand(shooter::stepPowerUp, shooter));
    shooterController.povLeft().onTrue(new InstantCommand(shooter::stepPowerDown, shooter));
    shooterController.povDown().onTrue(new InstantCommand(shooter::stopFlywheelPower, shooter));
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("IntakeOn",
        Commands.startEnd(arm::spinOn, arm::spinOff, arm));
    NamedCommands.registerCommand("IntakeReverse",
        Commands.startEnd(arm::spinReverse, arm::spinOff, arm));
    NamedCommands.registerCommand("IntakeOff",
        Commands.runOnce(arm::spinOff, arm));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

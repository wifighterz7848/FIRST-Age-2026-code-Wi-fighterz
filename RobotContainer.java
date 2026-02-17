package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.pathplanner.lib.auto.AutoBuilder;



public final class RobotContainer {

    /* ---------------- PathPlanner Auto Chooser ---------------- */

private final SendableChooser<Command> autoChooser;
private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");


    /* ---------------- Auto-aim tuning ---------------- */

    private static final String LIMELIGHT_LEFT = "limelight-left";
    private static final String LIMELIGHT_RIGHT = "limelight-right";

    // CHANGE THESE TO MATCH YOUR REAL MOUNT ANGLES (sign might need flipping!)
    private static final double LEFT_CAM_YAW_OFFSET  = -30.0;
    private static final double RIGHT_CAM_YAW_OFFSET =  30.0;

    private static final double AIM_KP = 0.02;
    private static final double AIM_DEADBAND = 0.2;

    /* ---------------- Subsystems ---------------- */

    private final DriveSubsystem drive;




    /* ---------------- Controllers ---------------- */

    private final CommandXboxController driverController =
            new CommandXboxController(OIConstants.kDriverControllerPort);

    /* ---------------- Drive State ---------------- */

    private double swerveSpeedScaleTranslation = 0.1;
    private double swerveSpeedScaleRotation = 0.25;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        SignalLogger.enableAutoLogging(false);

        drive = DriveSubsystem.getInstance();

        autoChooser = AutoBuilder.buildAutoChooser(); // auto-populated from deploy autos :contentReference[oaicite:5]{index=5}

autoTab.add("Auto Chooser", autoChooser)
       .withSize(2, 1)
       .withPosition(0, 0);


        // IMPORTANT:
        // DO NOT call teleopInit() here. Call robotContainer.teleopInit()
        // from Robot.java's teleopInit() instead.
        configureButtonBindings();
    }

    /* ---------------- Speed Modes ---------------- */

    private void setSlowSpeed() {
        swerveSpeedScaleTranslation = 0.06;
        swerveSpeedScaleRotation = 0.04;
    }

    private void setNormalSpeed() {
        swerveSpeedScaleTranslation = 0.4;
        swerveSpeedScaleRotation = 0.3;
    }

    /* ---------------- Teleop ---------------- */

    // Call this from Robot.java -> teleopInit()
    public void teleopInit() {
        setNormalSpeed();
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
                    rotation = cookRawJoystickInput(
                        driverController.getHID().getRawAxis(4),
                        -swerveSpeedScaleRotation
                    );
                }

                drive.drive(
                    cookRawJoystickInput(
                        driverController.getHID().getRawAxis(1),
                        -swerveSpeedScaleTranslation),
                    cookRawJoystickInput(
                        driverController.getHID().getRawAxis(0),
                        -swerveSpeedScaleTranslation),
                    rotation,
                    0,
                    ybump
                );

            }, drive)
        );

        driverController.rightTrigger(0.5)
            .whileTrue(new RunCommand(drive::setX, drive));

        // CHANGED: onTrue so it doesn't spam every scheduler loop
        driverController.rightStick()
            .onTrue(new InstantCommand(drive::resetFieldOrientedDir, drive));

        driverController.leftBumper()
            .onTrue(new InstantCommand(this::setSlowSpeed))
            .onFalse(new InstantCommand(this::setNormalSpeed));



    }

    /* ---------------- Autonomous ---------------- */

    public Command getAutonomousCommand() {
        return null;
    }

    /* ---------------- Limelight Dashboard ---------------- */

    private void updateLimelightDashboard() {

        // Left
        SmartDashboard.putBoolean(
            "LL Left Has Target",
            LimelightHelpers.getTV(LIMELIGHT_LEFT));

        SmartDashboard.putNumber(
            "LL Left TX Raw",
            LimelightHelpers.getTX(LIMELIGHT_LEFT));

        // Right
        SmartDashboard.putBoolean(
            "LL Right Has Target",
            LimelightHelpers.getTV(LIMELIGHT_RIGHT));

        SmartDashboard.putNumber(
            "LL Right TX Raw",
            LimelightHelpers.getTX(LIMELIGHT_RIGHT));

        // Corrected values (robot-ish frame)
        double midLeft =
            getMidFromCamera(LIMELIGHT_LEFT, LEFT_CAM_YAW_OFFSET);

        double midRight =
            getMidFromCamera(LIMELIGHT_RIGHT, RIGHT_CAM_YAW_OFFSET);

        SmartDashboard.putNumber("LL Mid Left (Robot)", midLeft);
        SmartDashboard.putNumber("LL Mid Right (Robot)", midRight);
    }

    /* ---------------- Auto Aim ---------------- */

    private double getAutoAimRotation() {

        double txLeft =
            getMidFromCamera(LIMELIGHT_LEFT, LEFT_CAM_YAW_OFFSET);

        double txRight =
            getMidFromCamera(LIMELIGHT_RIGHT, RIGHT_CAM_YAW_OFFSET);

        boolean leftValid = !Double.isNaN(txLeft);
        boolean rightValid = !Double.isNaN(txRight);

        SmartDashboard.putBoolean("AutoAim Left Valid", leftValid);
        SmartDashboard.putBoolean("AutoAim Right Valid", rightValid);

        // No targets
        if (!leftValid && !rightValid) {
            SmartDashboard.putString("AutoAim Source", "NONE");
            SmartDashboard.putNumber("AutoAim TX Mid", Double.NaN);
            SmartDashboard.putNumber("AutoAim Rotation Cmd", 0.0);
            return 0.0;
        }

        // Fuse cameras
        double txMid;
        if (leftValid && rightValid) {
            txMid = (txLeft + txRight) / 2.0;
            SmartDashboard.putString("AutoAim Source", "FUSED");
        } else if (leftValid) {
            txMid = txLeft;
            SmartDashboard.putString("AutoAim Source", "LEFT");
        } else {
            txMid = txRight;
            SmartDashboard.putString("AutoAim Source", "RIGHT");
        }

        SmartDashboard.putNumber("AutoAim TX Mid", txMid);

        // Deadband
        if (Math.abs(txMid) < AIM_DEADBAND) {
            SmartDashboard.putNumber("AutoAim Rotation Cmd", 0.0);
            return 0.0;
        }

        // P control
        double rotation = -txMid * AIM_KP;

        rotation = MathUtil.clamp(rotation, -0.25, 0.25);
        SmartDashboard.putNumber("AutoAim Rotation Cmd", rotation);

        return rotation;
    }

    /* ---------------- Tag Midpoint From One Camera ---------------- */
    /* UPDATED: null guard + one-tag fallback so it works in real matches */

    private double getMidFromCamera(String name, double yawOffset) {

        LimelightHelpers.RawFiducial[] fiducials =
                LimelightHelpers.getRawFiducials(name);

        if (fiducials == null || fiducials.length == 0) {
            return Double.NaN;
        }

        boolean found9 = false;
        boolean found10 = false;

        double tx9 = 0.0;
        double tx10 = 0.0;

        for (LimelightHelpers.RawFiducial f : fiducials) {

            if (f.id == 9) {
                tx9 = f.txnc + yawOffset;
                found9 = true;
            } else if (f.id == 10) {
                tx10 = f.txnc + yawOffset;
                found10 = true;
            }
        }

        // Both tags -> midpoint
        if (found9 && found10) {
            return (tx9 + tx10) / 2.0;
        }

        // Fallback: if only one tag, aim at that tag
        if (found9) return tx9;
        if (found10) return tx10;

        return Double.NaN;
    }
}

    public void startCamera() {
        CameraServer.startAutomaticCapture().setResolution(320, 180);
    }
}


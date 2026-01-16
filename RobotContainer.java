// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;  // ADD THIS LINE

public final class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem drive;
    private final VisionSubsystem m_visionSubsystem;  // ADD THIS LINE

    // Controllers
    CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    Joystick controller1 = new Joystick(1);
    Joystick controller2 = new Joystick(2);
    double swerveSpeedScaleTranslation = 0;
    double swerveSpeedScaleRotation = 0;
    boolean startPressedOnce = false;

    void setSlowSpeed() {
        if (startPressedOnce) {
            swerveSpeedScaleTranslation = 0.06;
            swerveSpeedScaleRotation = 0.040;
        }
    };

    void setNormalSpeed() {
        if (startPressedOnce) {
            swerveSpeedScaleTranslation = 0.15;
            swerveSpeedScaleRotation = 0.15;
        }
    }

    void setTurboSpeed() {
        if (startPressedOnce) {
            swerveSpeedScaleTranslation = 1;
            swerveSpeedScaleRotation = 1;
        }
    }
    
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        SignalLogger.enableAutoLogging(false);

        drive = DriveSubsystem.getInstance();
        
        // ADD THIS LINE - Initialize vision subsystem
        m_visionSubsystem = new VisionSubsystem(drive);
      
        // Robot won't drive until Start button pressed.
        teleopInit();

        // Configure the button bindings
        configureButtonBindings();
    }

    // Cooks the raw Joystick analog stick value.
    private double cookRawJoystickInput(double v, double scale) {
        double vr = MathUtil.applyDeadband(v, OIConstants.kDriveDeadband);
        vr = vr * vr * vr;
        return MathUtil.clamp(vr, -1.0, 1.0) * scale;
    }

    public void teleopInit() {
        System.out.print("Robot.teleopInit\n");
        startPressedOnce = true;
        setNormalSpeed();
    }

    private void configureButtonBindings() {
        // Driver - Default command (Swerve Drive)
        drive.setDefaultCommand(
                new RunCommand(
                        () -> { 
                                double ybump = 0.0;
                                if (driverController.x().getAsBoolean()) {
                                    ybump += 0.025;
                                }
                                if (driverController.b().getAsBoolean()) {
                                    ybump -= 0.025;
                                }

                                drive.drive(
                                cookRawJoystickInput(driverController.getRawAxis(1), -swerveSpeedScaleTranslation),
                                cookRawJoystickInput(driverController.getRawAxis(0), -swerveSpeedScaleTranslation),
                                cookRawJoystickInput(driverController.getRawAxis(4), -swerveSpeedScaleRotation),
                                0, ybump);
                        },
                        drive));

        // Driver - Apply Brakes
        driverController.rightTrigger(0.5)
                .whileTrue(new RunCommand(
                        () -> drive.setX(),
                        drive));

        // Driver - Reset Field Oriented Dir
        driverController.rightStick().whileTrue(new InstantCommand(() -> drive.resetFieldOrientedDir(), drive));

        // Driver - Slow Button
        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> { setSlowSpeed(); }))
            .onFalse(new InstantCommand(() -> { setNormalSpeed(); }));

        // Driver - Turbo Button
        driverController.rightBumper()
            .onTrue(new InstantCommand(() -> { setTurboSpeed(); }))
            .onFalse(new InstantCommand(() -> { setNormalSpeed(); }));
        
        // ADD THIS - Vision Debug Button (Y button)
        driverController.y()
            .onTrue(new InstantCommand(() -> {
                System.out.println("=== Vision Debug ===");
                System.out.println("Current Pose: " + drive.getPose());
                System.out.println("Tags visible: " + m_visionSubsystem.getTagCount());
                System.out.println("Has target: " + m_visionSubsystem.hasTarget());
                if (m_visionSubsystem.hasTarget()) {
                    System.out.println("Vision Pose: " + m_visionSubsystem.getEstimatedPose());
                }
            }));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void startCamera() {
        CameraServer.startAutomaticCapture().setResolution(320, 180);
    }
}


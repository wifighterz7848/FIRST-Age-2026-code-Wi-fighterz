// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
/*
waypoints = PathPlannerPath.waypointsFromPoses(
            getPoseEstimate(),
            new Pose2d(3, 4.01, Rotation2d.fromDegrees(300-180))
        );
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

                path.preventFlipping = true;
                
                return new FollowPathCommand(
                    path, 
                    this::getPoseEstimate,
                    this::getChassisSpeeds, 
                    (speeds, feedforwards) -> autoDrive(speeds),
                    new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0) 
                        ), 
                    Constants.Swerve.robotConfig, 
                    () -> false, 
                    this);
                    
    }
 */

public final class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  //private final Pigeon2 m_pigeon2 = new Pigeon2(20, "rio");

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator m_odometry;

  private Rotation2d m_gyro_field_forward;
	  
  private final Field2d m_field = new Field2d();
	  
  private boolean m_is_alliance_known = false;
  private boolean m_is_red_alliance = false;

  // PathPlanner configuration
  // public static final RobotConfig ppConfig =
  //     new RobotConfig(
  //       74.088,   // Kg
  //       6.883,      // MOI
  //         new ModuleConfig(
  //           Constants.ModuleConstants.kWheelDiameterMeters/2.0,
  //             maxSpeedMetersPerSec,
  //             1.2,    // Wheel COF
  //             driveGearbox.withReduction(driveMotorReduction),
  //             driveMotorCurrentLimit,
  //             1),
  //             DriveConstants.kDriveKinematics);  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    
    // Pigeon2Configurator cfg = m_pigeon2.getConfigurator();
    // Pigeon2Configuration config = new Pigeon2Configuration();
    // cfg.apply(config.Pigeon2Features.withEnableCompass(false));
    // // Reset heading in gyro. This really doesn't mean or do anything, since we don't know where the robot is facing.
    //   m_pigeon2.reset();

    // Assume we start facing the driver
    Pose2d startingPose;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      startingPose = new Pose2d(new Translation2d(0, 0),Rotation2d.fromDegrees(0));
    } else {
      startingPose = new Pose2d(new Translation2d(0, 0),Rotation2d.fromDegrees(180));
    }
    	
    m_odometry = new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          getGyroHeading2d(),
          getModulePositions(),
          startingPose,
          DriveConstants.stateStdDevs,
          Constants.Limelight.visionMeasurementStdDevs
      );

    // Calculate the field to gyro offset for field oriented drive
    setEstimatedPose(startingPose);

    SmartDashboard.putData("Field", m_field);
  }

  public boolean initPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return false;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getEstimatedPose, // Robot pose supplier
        this::setEstimatedPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> pathplannerDriveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                      // holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
              //this.m_lights.setTeamRed();
              return true;
            } else {
              //this.m_lights.setTeamBlue();
              return false;
            }
        }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    return true;
  }

  /**
   * Returns an instance of the class.
   * @return an instance of the class
   */
  static DriveSubsystem instance = null;
  public static DriveSubsystem getInstance()
  {
    if(instance == null)
    {
      instance = new DriveSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {


    if ( !m_is_alliance_known ) {
        setX();
        return;
    }
    // Update the odometry/pose in the periodic block
    m_odometry.updateWithTime(Timer.getFPGATimestamp(),
        getGyroHeading2d(),
        getModulePositions()
        );
    Pose2d est_pose = getEstimatedPose();
    m_field.setRobotPose(est_pose); //field2_pose);

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  private Pose2d getEstimatedPose() {
    return m_odometry.getEstimatedPosition();     //.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  private void setEstimatedPose(Pose2d pose) 
  {
    Rotation2d gyroHeading = getGyroHeading2d();
    String ally;
    Rotation2d curGyro = gyroHeading.plus(pose.getRotation());    // Convert the gyro to blue field coordinates
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      //reverse for Red
      ally = "RED";
      m_gyro_field_forward = curGyro.minus(Rotation2d.fromDegrees(180.0));
    } else {
      ally = "BLUE";
      m_gyro_field_forward = curGyro;
    }

    System.out.print(String.format("Gyro offset after %s setEstimatedPose: %f degrees\n", ally, m_gyro_field_forward.getDegrees() ));
    
    m_odometry.resetPosition(
      gyroHeading,
        getModulePositions(),
        pose);
  }

  /**
 * Adds a vision measurement to the pose estimator
 * @param visionPose The pose from vision
 * @param timestamp The timestamp of the vision measurement in seconds
 */
public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_odometry.addVisionMeasurement(visionPose, timestamp);
}

/**
 * Gets the current robot pose estimate
 * @return Current estimated pose
 */
public Pose2d getPose() {
    return getEstimatedPose();
}

  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed_field  Speed of the robot in the x direction (forward).
   * @param ySpeed_field  Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param xSpeed_robot  Speed of the robot in the x direction (forward).
   * @param ySpeed_robot  Speed of the robot in the y direction (sideways).
   */
  public void drive(double xSpeed_field, double ySpeed_field, double rot, double xSpeed_robot, double ySpeed_robot ) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double ySpeedDelivered_field = ySpeed_field * DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeedDelivered_field = xSpeed_field * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered_robot = ySpeed_robot * DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeedDelivered_robot = xSpeed_robot * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    
    Rotation2d gyroRotationToField = getGyroHeading2d().minus(m_gyro_field_forward);
    final ChassisSpeeds targetSpeeds_field = 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered_field, ySpeedDelivered_field, rotDelivered, gyroRotationToField );

    final ChassisSpeeds targetSpeeds_robot = 
      new ChassisSpeeds(xSpeedDelivered_robot, ySpeedDelivered_robot, 0.0);

    final ChassisSpeeds targetSpeeds = targetSpeeds_field.plus(targetSpeeds_robot); 

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * The autonomous method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.
   * Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void auto_drive(Translation2d translation, double rotation, boolean fieldRelative) {
      // swerveDrive.drive(translation,
      //     rotation,
      //     fieldRelative,
      //     false); // Open loop is disabled since it shouldn't be used most of the time.
      double maxSpeed = 0.05, maxRot = 0.10;
      double tx = MathUtil.clamp(translation.getX(), -maxSpeed, maxSpeed);
      double ty = MathUtil.clamp(translation.getY(), -maxSpeed, maxSpeed);
      double rot = MathUtil.clamp(rotation, -maxRot, maxRot);
      
      if (fieldRelative)  {
        drive( tx, ty, rot, 0, 0 );
      } else {
        drive( 0, 0, rot, tx, ty );
        
      }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    SwerveModuleState[] targetStates = new SwerveModuleState[4];
    targetStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    targetStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    targetStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    targetStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    setModuleStates(targetStates);
  }

  private void pathplannerDriveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds( desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
              m_frontLeft.getState(),
              m_frontRight.getState(),
              m_rearLeft.getState(),
              m_rearRight.getState()
    };
  }
  
  /**
   * Get current module positions
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   m_frontLeft.resetEncoders();
  //   m_rearLeft.resetEncoders();
  //   m_frontRight.resetEncoders();
  //   m_rearRight.resetEncoders();
  // }

  /** Zeroes the heading of the robot for field oriented drive.
   *  For field-oriented drive, getGyroHeading2d.getDegrees() should be 0 when facing 
   *  away from your alliance wall (driver's station).
   */
  public void resetFieldOrientedDir() {
    //m_gyro.reset();
    //m_pigeon2.reset();
    m_gyro_field_forward = getGyroHeading2d();
    System.out.print(String.format("Gyro offset after reset: %f degrees\n", m_gyro_field_forward.getDegrees() ));
  }

  /**
   * Returns the heading of the robot.
   * For field-oriented drive, getGyroHeading2d should be 0 when facing 
   * away from your alliance wall (driver's station).
   * @return the robot's heading in degrees, from -180 to 180
   */
  private Rotation2d getGyroHeading2d() {
    Rotation2d newr = Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
    //Rotation2d newr = m_pigeon2.getRotation2d();
    //xreturn Rotation2d.fromDegrees(m_pigeon2.getAngle());
    return newr;
  }
    

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  //   //return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  //   return m_pigeon2.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }
}

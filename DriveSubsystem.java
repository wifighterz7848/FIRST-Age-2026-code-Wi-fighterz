// DriveSubsystem.java (WORKING “JITTERY” VERSION)
// - Keeps your pose estimator + LL vision
// - Keeps your driver field-oriented reset logic
// - No getAbsoluteHeading() needed for the jittery RobotContainer above
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
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
import frc.robot.LimelightHelpers;

public final class DriveSubsystem extends SubsystemBase {

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

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private static final String LIMELIGHT_NAME = "limelight-right";

  private final SwerveDrivePoseEstimator m_odometry;

  // Driver “field forward” offset
  private Rotation2d m_gyro_field_forward;

  private final Field2d m_field = new Field2d();

  private boolean m_is_alliance_known = false;
  private boolean m_is_red_alliance = false;

  static DriveSubsystem instance = null;
  public static DriveSubsystem getInstance() {
    if (instance == null) instance = new DriveSubsystem();
    return instance;
  }

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    Pose2d startingPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

    m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroHeading2d(),
        getModulePositions(),
        startingPose,
        DriveConstants.stateStdDevs,
        Constants.Limelight.visionMeasurementStdDevs
    );

    m_gyro_field_forward = getGyroHeading2d();
    setEstimatedPose(startingPose);

    SmartDashboard.putData("Field", m_field);

    boolean ppOk = initPathPlanner();
    if (ppOk) {
      DriverStation.reportWarning("PathPlanner init OK (AutoBuilder configured).", false);
    } else {
      DriverStation.reportError("PathPlanner init FAILED (AutoBuilder NOT configured). Autos disabled.", false);
    }
  }

  public boolean initPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return false;
    }

    AutoBuilder.configure(
        this::getEstimatedPose,
        this::setEstimatedPose,
        this::getSpeeds,
        (speeds, feedforwards) -> pathplannerDriveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () -> {
          var a = DriverStation.getAlliance();
          return a.isPresent() && a.get() == DriverStation.Alliance.Red;
        },
        this
    );

    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> m_field.getObject("path").setPoses(poses)
    );

    return true;
  }

  @Override
  public void periodic() {

    if (!m_is_alliance_known) {
      var a = DriverStation.getAlliance();
      if (a.isPresent()) {
        m_is_alliance_known = true;
        m_is_red_alliance = (a.get() == Alliance.Red);
        DriverStation.reportWarning("Alliance detected: " + (m_is_red_alliance ? "RED" : "BLUE"), false);
      }
    }

    m_odometry.updateWithTime(
        Timer.getFPGATimestamp(),
        getGyroHeading2d(),
        getModulePositions()
    );

    addLimelightVisionMeasurement();

    Pose2d est_pose = getEstimatedPose();
    m_field.setRobotPose(est_pose);
    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("Gyro Heading Used", getGyroHeading2d().getDegrees());
    SmartDashboard.putNumber("Field Forward Offset", m_gyro_field_forward.getDegrees());
  }

  private Pose2d getEstimatedPose() {
    return m_odometry.getEstimatedPosition();
  }

  private void setEstimatedPose(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroHeading2d(),
        getModulePositions(),
        pose
    );
  }

  public Pose2d getPose() {
    return getEstimatedPose();
  }

  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(double xSpeed_field, double ySpeed_field, double rot,
                    double xSpeed_robot, double ySpeed_robot) {

    double ySpeedDelivered_field = ySpeed_field * DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeedDelivered_field = xSpeed_field * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered_robot = ySpeed_robot * DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeedDelivered_robot = xSpeed_robot * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    Rotation2d gyroRotationToField = getGyroHeading2d().minus(m_gyro_field_forward);

    final ChassisSpeeds targetSpeeds_field =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered_field, ySpeedDelivered_field, rotDelivered, gyroRotationToField
        );

    final ChassisSpeeds targetSpeeds_robot =
        new ChassisSpeeds(xSpeedDelivered_robot, ySpeedDelivered_robot, 0.0);

    final ChassisSpeeds targetSpeeds = targetSpeeds_field.plus(targetSpeeds_robot);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(swerveModuleStates);
  }

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

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public void resetFieldOrientedDir() {
    m_gyro_field_forward = getGyroHeading2d();
  }

  private Rotation2d getGyroHeading2d() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY));
  }

  private void addLimelightVisionMeasurement() {
    LimelightHelpers.PoseEstimate est =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    if (est == null) return;
    if (est.tagCount < 1) return;
    if (Double.isNaN(est.pose.getX()) || Double.isNaN(est.pose.getY())) return;

    double ts = est.timestampSeconds;
    double now = Timer.getFPGATimestamp();
    if (now - ts > 1.0) return;

    double xyStd = (est.tagCount >= 2) ? 0.25 : 0.75;
    double thStd = Math.toRadians(10); // original “jittery” behavior: vision can influence heading a bit

    m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStd, xyStd, thStd));
    m_odometry.addVisionMeasurement(est.pose, ts);
  }
}

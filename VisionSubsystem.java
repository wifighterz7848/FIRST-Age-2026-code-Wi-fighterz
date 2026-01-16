package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;
    private final String m_limelightName = "limelight";
    
    private PoseEstimate m_lastEstimate = null;
    
    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    }
    
    @Override
    public void periodic() {
        // Update robot orientation for MegaTag2 - CRITICAL!
        double yaw = m_driveSubsystem.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(m_limelightName, yaw, 0, 0, 0, 0, 0);
        
        // Get the latest MegaTag2 result from Limelight
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
        
        // Store it for getter methods
        m_lastEstimate = mt2;
        
        // Debug - print to console every 50 loops (once per second)
        if (m_driveSubsystem != null) {
            // Uncomment this for debugging:
            // if (java.lang.Math.random() < 0.02) {  // Print occasionally
            //     System.out.println("Vision: mt2=" + (mt2 != null ? "exists" : "null") + 
            //                        ", tags=" + (mt2 != null ? mt2.tagCount : "N/A"));
            // }
        }
        
        // Check if we have valid vision data
        if (mt2 != null && mt2.tagCount > 0) {
            // Add vision measurement to odometry
            m_driveSubsystem.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            
            // Update SmartDashboard with vision info
            SmartDashboard.putNumber("Vision/Tags Seen", mt2.tagCount);
            SmartDashboard.putNumber("Vision/Avg Distance", mt2.avgTagDist);
            SmartDashboard.putNumber("Vision/Tag Span", mt2.tagSpan);
            SmartDashboard.putNumber("Vision/X", mt2.pose.getX());
            SmartDashboard.putNumber("Vision/Y", mt2.pose.getY());
            SmartDashboard.putNumber("Vision/Rotation", mt2.pose.getRotation().getDegrees());
            SmartDashboard.putBoolean("Vision/Has Valid Data", true);
            
            // Also put raw Limelight values for debugging
            SmartDashboard.putBoolean("Vision/LL Has Target", LimelightHelpers.getTV(m_limelightName));
            SmartDashboard.putNumber("Vision/LL TX", LimelightHelpers.getTX(m_limelightName));
            SmartDashboard.putNumber("Vision/LL TY", LimelightHelpers.getTY(m_limelightName));
            
        } else {
            SmartDashboard.putNumber("Vision/Tags Seen", 0);
            SmartDashboard.putBoolean("Vision/Has Valid Data", false);
            
            // Still show if Limelight sees ANYTHING (even if not valid pose)
            SmartDashboard.putBoolean("Vision/LL Has Target", LimelightHelpers.getTV(m_limelightName));
        }
    }
    
    /**
     * Returns whether the Limelight currently has any AprilTag targets
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(m_limelightName);
    }
    
    /**
     * Gets the current pose estimate from vision
     */
    public Pose2d getEstimatedPose() {
        if (m_lastEstimate != null) {
            return m_lastEstimate.pose;
        }
        return new Pose2d();
    }
    
    /**
     * Gets the number of AprilTags currently visible
     */
    public int getTagCount() {
        if (m_lastEstimate != null) {
            return m_lastEstimate.tagCount;
        }
        return 0;
    }
    
    /**
     * Gets the last valid pose estimate (for debugging)
     */
    public PoseEstimate getLastEstimate() {
        return m_lastEstimate;
    }
}
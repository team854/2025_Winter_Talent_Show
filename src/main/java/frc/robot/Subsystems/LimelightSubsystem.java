package frc.robot.Subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Libraries.LimelightHelpers;
import frc.robot.Libraries.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase{
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private PoseEstimate limelightPoseEstimate = new PoseEstimate();
   
    public LimelightSubsystem() {
        gyro.zeroYaw();
    }

    private void updateVisionEstimate() {
        LimelightHelpers.SetRobotOrientation(Constants.LimelightConstants.LIMELIGHT_NAME, (double) gyro.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);

        try {
            PoseEstimate tempLimelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightConstants.LIMELIGHT_NAME);
            //System.out.println(tempLimelightPoseEstimate.tagCount);
            if (tempLimelightPoseEstimate != null && tempLimelightPoseEstimate.pose != null) {
                limelightPoseEstimate = tempLimelightPoseEstimate;
            }
        }
        catch (Exception e) {
            System.err.println("An error occurred: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        updateVisionEstimate();

        SmartDashboard.putNumberArray("Limelight/Bot Pose" ,LimelightHelpers.pose3dToArray(getTargetPose()));
    }

    public Pose3d getTargetPose() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LimelightConstants.LIMELIGHT_NAME);
    }

    public PoseEstimate getPoseEstimate() {
        return limelightPoseEstimate;
    }
}

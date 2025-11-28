package frc.robot.Subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
    }

    public PoseEstimate getPoseEstimate() {
        return limelightPoseEstimate;
    }
}

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(Constants.DriveConstants.LEFT_MOTOR_1_ID);

    private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_MOTOR_1_ID);

    public DriveSubsystem() {
        // Inverts one sides (Depends on how the gearbox is set up)
        leftMotor1.setInverted(Constants.DriveConstants.LEFT_MOTORS_REVERSED);

        rightMotor1.setInverted(Constants.DriveConstants.RIGHT_MOTORS_REVERSED);

        // Makes it brake if it is not moving
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
    }

    public void setDriveMotors(double leftMotor, double rightMotor) {
        leftMotor1.set(ControlMode.PercentOutput, leftMotor);
        rightMotor1.set(ControlMode.PercentOutput, rightMotor);
    }
}

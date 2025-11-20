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
    private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_1_ID);
    private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(Constants.DriveConstants.LEFT_MOTOR_2_ID);

    private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_1_ID);
    private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_MOTOR_2_ID);

    private final Pigeon2 gyro = new Pigeon2(1, "rio");

    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(
        Constants.DriveConstants.WHEEL_BASE_WIDTH.in(Meters)
    );

    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
        driveKinematics,
        gyro.getRotation2d(),
        0,
        0,
        new Pose2d());

    public DriveSubsystem() {
        // Inverts one sides (Depends on how the gearbox is set up)
        leftMotor1.setInverted(Constants.DriveConstants.LEFT_MOTORS_REVERSED);
        leftMotor2.setInverted(Constants.DriveConstants.LEFT_MOTORS_REVERSED);

        rightMotor1.setInverted(Constants.DriveConstants.RIGHT_MOTORS_REVERSED);
        rightMotor2.setInverted(Constants.DriveConstants.RIGHT_MOTORS_REVERSED);

        // Makes it brake if it is not moving
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Brake);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Brake);

        // Makes one of the motors follow the other
        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
    }

    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    public void setDriveMotors(double leftMotor, double rightMotor) {
        leftMotor1.set(ControlMode.PercentOutput, leftMotor);
        rightMotor1.set(ControlMode.PercentOutput, rightMotor);
    }
}

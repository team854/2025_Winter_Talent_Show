package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(Constants.DriveConstants.LEFT_MOTOR_1_ID);
    private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_2_ID);

    private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_MOTOR_1_ID);
    private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_2_ID);
    
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

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

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Angle getGyroFacing() {
        return Degree.of(MathUtil.inputModulus(gyro.getAngle(), 0, 360));
    }

    public void setDriveMotors(double leftMotor, double rightMotor) {
        leftMotor1.set(ControlMode.PercentOutput, leftMotor);
        rightMotor1.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive/Gyro", getGyroFacing().in(Degree));
    }
}

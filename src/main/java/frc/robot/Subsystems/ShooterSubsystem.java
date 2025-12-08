package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Main;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMaxConfig shooterConfig1 = new SparkMaxConfig();
    private final SparkMaxConfig shooterConfig2 = new SparkMaxConfig();
    private final SparkMax shooterMotor1 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID_1,MotorType.kBrushless);
    private final SparkMax shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID_2,MotorType.kBrushless);
    private AngularVelocity targetSpeed = RotationsPerSecond.of(0);

    private final SparkClosedLoopController shooterController1 = shooterMotor1.getClosedLoopController();
    private final SparkClosedLoopController shooterController2 = shooterMotor2.getClosedLoopController();


    public ShooterSubsystem(){
        shooterConfig1.idleMode(IdleMode.kCoast);
        shooterConfig1.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.ShooterConstants.KP)
            .i(Constants.ShooterConstants.KI)
            .d(Constants.ShooterConstants.KD)
            .outputRange(-1, 1);
        shooterConfig1.closedLoop.maxMotion
            .maxVelocity(Constants.ShooterConstants.SHOOTER_MAX_ANGULAR_VELOCITY.in(RPM))
            .maxAcceleration(Constants.ShooterConstants.SHOOTER_MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond))
            .allowedClosedLoopError(Constants.ShooterConstants.SHOOTER_ALLOWED_ERROR.in(RPM));

        shooterConfig2.idleMode(IdleMode.kCoast);
        shooterConfig2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.ShooterConstants.KP)
            .i(Constants.ShooterConstants.KI)
            .d(Constants.ShooterConstants.KD)
            .outputRange(-1, 1);
        shooterConfig2.closedLoop.maxMotion
            .maxVelocity(Constants.ShooterConstants.SHOOTER_MAX_ANGULAR_VELOCITY.in(RPM))
            .maxAcceleration(Constants.ShooterConstants.SHOOTER_MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond))
            .allowedClosedLoopError(Constants.ShooterConstants.SHOOTER_ALLOWED_ERROR.in(RPM));

        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
     public void setSpeed(AngularVelocity speed) {
        targetSpeed = speed;
        shooterController1.setReference(speed.in(RPM), ControlType.kMAXMotionVelocityControl);
        shooterController2.setReference(speed.in(RPM), ControlType.kMAXMotionVelocityControl);

        SmartDashboard.putNumber("Shooter/Target RPM", speed.in(RPM));
    }

    public AngularVelocity getMotor1CurrentSpeed() {
        return RPM.of(shooterMotor1.getEncoder().getVelocity());
    }

    public AngularVelocity getMotor2CurrentSpeed() {
        return RPM.of(shooterMotor2.getEncoder().getVelocity());
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Motor 1 RPM", getMotor1CurrentSpeed().in(RPM));
        SmartDashboard.putNumber("Shooter/Motor 2 RPM", getMotor2CurrentSpeed().in(RPM));
    }


}

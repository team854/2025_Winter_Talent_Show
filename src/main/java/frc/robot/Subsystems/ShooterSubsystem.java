package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
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
    //private final SparkMax shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID_2,MotorType.kBrushless);

    private final RelativeEncoder shootEncoder1 = shooterMotor1.getEncoder();

    private final PIDController pid = new PIDController(Constants.ShooterConstants.SHOOTER_P,
                                                        Constants.ShooterConstants.SHOOTER_I, 
                                                        Constants.ShooterConstants.SHOOTER_D);
    private AngularVelocity targetSpeed = RotationsPerSecond.of(0);


    public ShooterSubsystem(){
        shooterConfig1.idleMode(IdleMode.kBrake);
        

        /*
        shooterConfig2.idleMode(IdleMode.kBrake);
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
        */

        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
     public void setSpeed(AngularVelocity speed) {
        targetSpeed = speed;
        //shooterController2.setReference(speed.in(RPM), ControlType.kMAXMotionVelocityControl);

        SmartDashboard.putNumber("Shooter/Target RPM", speed.in(RPM));
    }

    public AngularVelocity getMotor1CurrentSpeed() {
        return RPM.of(shootEncoder1.getVelocity());
    }

    public AngularVelocity getMotor2CurrentSpeed() {
        //return RPM.of(shooterMotor2.getEncoder().getVelocity());
        return RPM.of(10);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }
    
    @Override
    public void periodic() {
        double pidOutput = MathUtil.clamp(pid.calculate(shootEncoder1.getVelocity(), targetSpeed.in(RPM)) / Constants.ShooterConstants.SHOOTER_DIVISOR, -0.1, 0.1);
        double finalMotorPower = pidOutput + (0.00016 * targetSpeed.in(RPM));
        shooterMotor1.set(finalMotorPower);

        SmartDashboard.putNumber("Shooter/Motor 1 RPM", getMotor1CurrentSpeed().in(RPM));
        SmartDashboard.putNumber("Shooter/Motor 2 RPM", getMotor2CurrentSpeed().in(RPM));
    }


}

package frc.robot.Subsystems;

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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Main;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    public final SparkMax shooterMotor = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR,MotorType.kBrushless);
    public final SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();
    private final double kP = Constants.ShooterConstants.KP;
    private final double kI = Constants.ShooterConstants.KI;
    private final double kD = Constants.ShooterConstants.KD;
    public ShooterSubsystem(){
        shooterConfig.idleMode(IdleMode.kBrake);
        shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(Constants.ShooterConstants.KP).i(Constants.ShooterConstants.KI).d(Constants.ShooterConstants.KD).outputRange(-1, 1);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
     public void setSpeed(double speed) {
        shooterController.setReference(speed, ControlType.kVelocity);
        
    }
    
  


}

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    
    private Angle armAngle;

    private final SparkMax armMotorObject = new SparkMax(Constants.ArmContants.ARM_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder armAbsoluteEncoder = armMotorObject.getAbsoluteEncoder();
    private final SparkMaxConfig armConfig = new SparkMaxConfig();

    private final PIDController pid = new PIDController(Constants.ArmContants.ARM_PID_P,
                                                        Constants.ArmContants.ARM_PID_I, 
                                                        Constants.ArmContants.ARM_PID_D);

    public ArmSubsystem()
    {
        armAngle = Degree.of(0);
        
        armConfig.inverted(Constants.ArmContants.ARM_MOTOR_INVERTED);
        armConfig.absoluteEncoder.positionConversionFactor(360);
        armConfig.absoluteEncoder.zeroOffset(Constants.ArmContants.ARM_ZERO_OFFSET);
        armConfig.absoluteEncoder.inverted(Constants.ArmContants.ARM_ENCODER_INVERTED);
        armMotorObject.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        pid.enableContinuousInput(0, 360);

        
    }

    @Override
    public void periodic() 
    {
        double stationaryAdditionAmount = Math.cos((armAbsoluteEncoder.getPosition() / 180.0) * Math.PI) * Constants.ArmContants.ARM_STATIONARY_CONSTANT;
        double pidOutput = pid.calculate(armAbsoluteEncoder.getPosition(), armAngle.in(Degree));
        armMotorObject.set(pidOutput + stationaryAdditionAmount);
    }

    public void setTargetArmAngle(Angle targetAngle)
    {
        double rawTargetAngle = targetAngle.in(Degree);
        rawTargetAngle = Math.max(rawTargetAngle, Constants.ArmContants.ARM_LOWER_LIMIT.in(Degree));
        rawTargetAngle = Math.min(rawTargetAngle, Constants.ArmContants.ARM_UPPER_LIMIT.in(Degree));

        armAngle = Degree.of(rawTargetAngle);
    }

    public Angle getTargetArmAngle()
    {
        return armAngle;
    }

    public Angle getCurrentArmAngle() {
        return Degree.of(armAbsoluteEncoder.getPosition());
    }

}

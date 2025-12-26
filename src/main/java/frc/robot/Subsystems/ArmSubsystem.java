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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    
    private Angle armAngle;

    private final SparkMax armMotorObject = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder armAbsoluteEncoder = armMotorObject.getAbsoluteEncoder();
    private final SparkMaxConfig armConfig = new SparkMaxConfig();

    private final PIDController pid = new PIDController(Constants.ArmConstants.ARM_PID_P,
                                                        Constants.ArmConstants.ARM_PID_I, 
                                                        Constants.ArmConstants.ARM_PID_D);

    public ArmSubsystem()
    {
        armAngle = Degree.of(0);
        
        armConfig.inverted(Constants.ArmConstants.ARM_MOTOR_INVERTED);
        armConfig.absoluteEncoder.positionConversionFactor(360);
        armConfig.absoluteEncoder.zeroOffset(Constants.ArmConstants.ARM_ZERO_OFFSET);
        armConfig.absoluteEncoder.inverted(Constants.ArmConstants.ARM_ENCODER_INVERTED);
        armMotorObject.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        pid.setTolerance(Constants.ArmConstants.ARM_ALLOWED_ERROR.in(Degree));
        pid.enableContinuousInput(0, 360);

        
    }

    @Override
    public void periodic() 
    {
        
        SmartDashboard.putNumber("Arm/Absolute Angle", getCurrentArmAngle().in(Degree));
        SmartDashboard.putNumber("Arm/Target Angle", getTargetArmAngle().in(Degree));

        double stationaryAdditionAmount = Math.cos(getCurrentArmAngle().in(Radian)) * Constants.ArmConstants.ARM_STATIONARY_CONSTANT;
        double pidOutput = pid.calculate(armAbsoluteEncoder.getPosition(), armAngle.in(Degree));
        double finalMotorPower = pidOutput + stationaryAdditionAmount;
        armMotorObject.set(finalMotorPower);

        SmartDashboard.putNumber("Arm/Motor Percent", finalMotorPower * 100.0);
        
    }

    public void setTargetArmAngle(Angle targetAngle)
    {
        double rawTargetAngle = targetAngle.in(Degree);
        rawTargetAngle = Math.max(rawTargetAngle, Constants.ArmConstants.ARM_LOWER_LIMIT.in(Degree));
        rawTargetAngle = Math.min(rawTargetAngle, Constants.ArmConstants.ARM_UPPER_LIMIT.in(Degree));

        armAngle = Degree.of(rawTargetAngle);
    }

    public Angle getTargetArmAngle()
    {
        return armAngle;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public Angle getCurrentArmAngle() {
        return Degree.of(armAbsoluteEncoder.getPosition());
    }

}

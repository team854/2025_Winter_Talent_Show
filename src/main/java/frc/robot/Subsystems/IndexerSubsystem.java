package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
      
    private final WPI_TalonSRX indexMotor = new WPI_TalonSRX(Constants.IndexerConstants.INDEX_MOTOR_ID);
    private double currentSpeed = 0;
    private final DigitalInput m_proximitySensor = new DigitalInput(Constants.IndexerConstants.INDEX_PROXIMITY_PORT);
    public boolean getProximity(){
        return m_proximitySensor.get();
    }

    public IndexerSubsystem(){
        indexMotor.setInverted(Constants.IndexerConstants.INDEX_MOTOR_INVERSED);
        indexMotor.setNeutralMode(NeutralMode.Coast);
    }
    


    public void setSpeed(double speed) {
        currentSpeed = speed;
        indexMotor.set(currentSpeed);
    }

    public double getSpeed(){
        return currentSpeed;
    }


}

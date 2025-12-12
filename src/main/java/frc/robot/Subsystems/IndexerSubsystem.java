package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
      
    private final WPI_TalonSRX indexMotor = new WPI_TalonSRX(Constants.IndexerConstants.INDEX_MOTOR_ID);
    private final DigitalInput proximitySensor = new DigitalInput(Constants.IndexerConstants.INDEX_PROXIMITY_PORT);
    private double currentPower = 0;

    public IndexerSubsystem(){
        indexMotor.setInverted(Constants.IndexerConstants.INDEX_MOTOR_INVERSED);
        indexMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    @Override
    public void periodic() 
    {
        SmartDashboard.putBoolean("Indexer/Note Detected", getProximity());
    }
    
    public boolean getProximity(){
        return !proximitySensor.get();
    }

    public void setPower(double power) {
        currentPower = power;
        indexMotor.set(currentPower);
    }

    public double getPower(){
        return currentPower;
    }


}

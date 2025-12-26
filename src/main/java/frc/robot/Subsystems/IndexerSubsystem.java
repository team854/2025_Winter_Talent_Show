package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
    private final WPI_TalonSRX indexMotor = new WPI_TalonSRX(Constants.IndexerConstants.INDEX_MOTOR_ID);
    
    private DigitalInput proximitysensor = new DigitalInput( Constants.IndexerConstants.INDEX_PROXIMITY_PORT);
    
    private double speed = 0;
      
    private final WPI_TalonSRX indexMotor = new WPI_TalonSRX(Constants.IndexerConstants.INDEX_MOTOR_ID);
    private final DigitalInput proximitySensor = new DigitalInput(Constants.IndexerConstants.INDEX_PROXIMITY_PORT);

    public IndexerSubsystem(){
        indexMotor.setInverted(Constants.IndexerConstants.INDEX_MOTOR_INVERSED);
        indexMotor.setNeutralMode(NeutralMode.Coast);
        
      
    }
    
    
    public void setSpeed(double speed){
        this.speed= speed;
        indexMotor.set(speed);
    }
    
    public double getSpeed(){
        return speed;
    }

   
    public boolean getProximity(){
        return proximitysensor.get();
    }
    

    @Override
    public void periodic() {
        boolean objectDetected = !proximitysensor.get(); 
        // Some sensors are active-low, so invert if needed
        if (objectDetected) {
            System.out.println("Object detected!");
        }else {System.out.println ("Object Not detected");}
    }
    


    

}

    }
    
    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Indexer/Motor Percent", getPower());
        SmartDashboard.putBoolean("Indexer/Note Detected", getProximity());
    }
    
    public boolean getProximity(){
        return !proximitySensor.get();
    }

    public void setPower(double power) {
        indexMotor.set(power);
    }

    public double getPower(){
        return indexMotor.get();
    }


}

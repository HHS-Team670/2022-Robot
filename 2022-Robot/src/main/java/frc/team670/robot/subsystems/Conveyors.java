package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.SpeedController;
import java.lang.AutoCloseable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import frc.team670.robot.constants.RobotMap;

public class Conveyors extends MustangSubsystemBase 
{


    private CANSparkMax roller;
    private int DeviceID = 1;

    private double conveyorSpeed; 

    public Conveyors(double speed) 
    {
        conveyorSpeed = speed;
        roller = new CANSparkMax(DeviceID, MotorType.kBrushless);

    }

    public void run(boolean reversed) 
    {
        if (reversed) {
            conveyorSpeed = Math.abs(conveyorSpeed) * -1;
        } else {
            conveyorSpeed = Math.abs(conveyorSpeed);
        }
        
        roller.set(conveyorSpeed);
    }

    public void disable() 
    {
        roller.disable();
    }

    public void stop() 
    {
        roller.stopMotor();
    }

    public void setSpeed(double speed)
    {
        conveyorSpeed = speed;
    }

    @Override
    public HealthState checkHealth() 
    {
        if ( (roller.getLastError() != null) && (roller.getLastError() != REVLibError.kOk) ) {
            return HealthState.RED;
        }
        
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() 
    {
        Logger.consoleLog("Speed: " + conveyorSpeed);
    } 

}

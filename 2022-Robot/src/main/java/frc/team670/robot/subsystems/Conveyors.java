package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;



public class Conveyors extends MustangSubsystemBase 
{

    public Conveyors(){
        Conveyor c1 = new Conveyor();
    }

}



public class Conveyor 
{

    private SparkMAXLite roller;

    private double conveyorSpeed;

    private boolean converyorState = false;
    

    BeamBreak beamBreak;

    public Conveyor(int id, MotorConfig.Motor_Type type, double speed) 
    {
        roller = new SparkMAXLite(id, type);

        conveyorSpeed = speed;

        beamBreak = new BeamBreak(0);

        numBalls = 0; //really need this?
    }

    //SENSOR SPECIAL FUNCTIONS
    

    // public void updateConveyorStates () 
    // {
    //     for (int i = 0; i < conveyorStates.length)
    // }

    public boolean active() 
    {
        return beamBreak.isTriggered();
    }



    //CONVERY SPECIAL FUNCTIONS !!!KEEP SEPERATE...
    public void run(boolean reversed) 
    {
        if (reversed) 
        {
            conveyorSpeed = Math.abs(conveyorSpeed) * -1;
        } else 
        {
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

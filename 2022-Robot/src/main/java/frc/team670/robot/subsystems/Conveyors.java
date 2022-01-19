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

    private SparkMAXLite roller;
    private double conveyorSpeed;

    private boolean conveyorStates[] = {false, false, false};
    private int numBalls=0;


    BeamBreak[] beamBreaks= new BeamBreak[3];
 
    public Conveyors(int id, MotorConfig.Motor_Type type, double speed) 
    {
        roller = new SparkMAXLite(id, type);
        conveyorSpeed = speed;
        //Check the dio port for the beamBreak sensors
        
        beamBreaks[0] = new BeamBreak(0);
        beamBreaks[1] = new BeamBreak(1);
        beamBreaks[2] = new BeamBreak(2);
        numBalls = 0;
        updateConveyorStates();
    }

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

    public void updateConveyorStates () {
        numBalls=0;
        for (int i = 0; i < conveyorStates.length;i++)
        {
            if(beamBreaks[i].isTriggered())
            {
                conveyorStates[i]=true;
                numBalls++;
            }else 
            {
                conveyorStates[i]=false;
            }


        }
    }

    public boolean isConveyorFull() {
        boolean conveyorCondition = false;
        if (beamBreaks[2].isTriggered())
            conveyorCondition = true;
        
        return conveyorCondition;
    }

    public void useConveyor () {
        if (!isConveyorFull()) {
            run(false);
        }
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

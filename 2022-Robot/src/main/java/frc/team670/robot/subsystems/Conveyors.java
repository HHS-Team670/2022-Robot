package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import frc.team670.robot.constants.RobotMap;

public class Conveyors extends MustangSubsystemBase 
{
    public Conveyor c1, c2;


    public Conveyors(){
        // c1 = new Conveyor();
        // c2 = new Conveyor();
    }

    // ACTIONS

    public void runConveyors(boolean intaking)
    {
        c1.run(intaking);
        c2.run(intaking);
        
        
    }
    public void setSpeed(double c1Speed, double c2Speed)
    {
        c1.setSpeed(c1Speed);
        c2.setSpeed(c2Speed);
    }

    public void stopAll()
    {
        c1.stop();
        c2.stop();
    }

    public boolean finished(){
        return !(!c1.isRunning && !c2.isRunning);
    
    }

    //DataCollection

    public int ballCount()
    {
        return c1.ball + c2.ball;
    }


    //MUSTANGESUBSYSTEM

    @Override
    public HealthState checkHealth() {
        if(c1.checkHealth()==HealthState.RED||c2.checkHealth()==HealthState.RED)
        {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        checkHealth();
        
    }





}



class Conveyor extends MustangSubsystemBase
{

    private SparkMAXLite roller;

    private double conveyorSpeed;

    private boolean conveyorState = false;
    public boolean isRunning = false;

    public int ball = 0;
    

    BeamBreak beamBreak;



    public Conveyor(int id, MotorConfig.Motor_Type type, double speed) 
    {
        
        roller=SparkMAXFactory.buildSparkMAX(id, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);

        conveyorSpeed = speed;

        beamBreak = new BeamBreak(0);

    }

    //SENSOR SPECIAL FUNCTIONS
    

    // public void updateConveyorStates () 
    // {
    //     for (int i = 0; i < conveyorStates.length)
    // }

    //Needs to be fixed. sensor number and placement has changed
    public boolean active() 
    {
        if(beamBreak.isTriggered())
        {
            ball = 1;
            conveyorState=true;
            return conveyorState;
        }

        ball = 0;
        conveyorState=false;

        return conveyorState;
        
    }



    //CONVERY SPECIAL FUNCTIONS !!!KEEP SEPERATE...
    public void run(boolean intaking) 
    {
        if (!intaking) 
        {
            conveyorSpeed = Math.abs(conveyorSpeed) * -1;
        } else 
        {
            conveyorSpeed = Math.abs(conveyorSpeed);
        }
        
        isRunning = true;
        roller.set(conveyorSpeed);
    }

    public void disable() 
    {
        isRunning = false;
        roller.disable();
    }

    public void stop() 
    {
        isRunning = false;
        roller.set(0);
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

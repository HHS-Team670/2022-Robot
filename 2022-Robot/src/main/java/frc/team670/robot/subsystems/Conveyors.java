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
    public void runConveyors(boolean intaking)
    {
        c1.run(intaking);
        c2.run(intaking);
    }
    public void stopAll()
    {
        c1.stop();
        c2.stop();
    }
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
            conveyorState=true;
            return conveyorState;
        }
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

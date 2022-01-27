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
    public Conveyor bottomConveyor, topConveyor;
 
 
    public Conveyors(){

    }
 
    // ACTIONS
 
    public void runConveyors(boolean intaking)
    {
        bottomConveyor.run(intaking);
        topConveyor.run(intaking);
    }
 
    public void stopAll()
    {
        bottomConveyor.stop();
        topConveyor.stop();
    }
    public void setSpeed(double bottomspeed, double topspeed)
    {
         bottomConveyor.setSpeed(bottomspeed);
         topConveyor.setSpeed(topspeed);
    }
 
 
    //DataCollection
 
    public int ballCount()
    {
        return bottomConveyor.ball + topConveyor.ball;
    }
 
 
    //MUSTANGESUBSYSTEM
 
    @Override
    public HealthState checkHealth() {
        if(bottomConveyor.checkHealth()==HealthState.RED||topConveyor.checkHealth()==HealthState.RED)
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
 
    public int ball = 0;
   
 
    BeamBreak beamBreak;
 
    double absConveyorSpeed;
 
    public Conveyor(int id, MotorConfig.Motor_Type type, double speed)
    {
       
        roller=SparkMAXFactory.buildSparkMAX(id, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
 
        conveyorSpeed = speed;
 
        beamBreak = new BeamBreak(0);

        absConveyorSpeed = Math.abs(conveyorSpeed);
 
    }
 
    //SENSOR SPECIAL FUNCTIONS
   
 
    // public void updateConveyorStates ()
    // {
    //     for (int i = 0; i < conveyorStates.length)
    // }
 
    //Needs to be fixed. sensor number and placement has changed
    public boolean running()
    {
        if(beamBreak.isTriggered())
        {
            ball = 1;
            conveyorState = true;
            return conveyorState;
        }
 
        ball = 0;
        conveyorState = false;
 
        return conveyorState;
       
    }
 
 
 
    //CONVERY SPECIAL FUNCTIONS !!!KEEP SEPERATE...
    public void run(boolean intaking)
    {
        if (!intaking)
        {
            conveyorSpeed = absConveyorSpeed * -1;
        } else
        {
            conveyorSpeed = absConveyorSpeed;
        }
       
        roller.set(conveyorSpeed);
    }
 
    public void disable()
    {
        roller.disable();
    }
 
    public void stop()
    {
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

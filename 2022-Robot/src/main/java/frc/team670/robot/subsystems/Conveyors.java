package frc.team670.robot.subsystems;
 
import com.revrobotics.REVLibError;
 
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
 
 
public class Conveyors extends MustangSubsystemBase
{
    public Conveyor primaryConveyor, secondaryConveyor;
 
 
    public Conveyors(){
        primaryConveyor = new Conveyor(RobotMap.PRIMARY_CONVEYOR_MOTOR);
        secondaryConveyor = new Conveyor(RobotMap.SECONDARY_CONVEYOR_MOTOR);
 
    }
 
    // ACTIONS
 
    public void runConveyors(boolean intaking)
    {
        primaryConveyor.run(intaking);
        secondaryConveyor.run(intaking);
    }
 
    public void stopAll()
    {
        primaryConveyor.stop();
        secondaryConveyor.stop();
    }
    public void setSpeed(double bottomspeed, double topspeed)
    {
         primaryConveyor.setSpeed(bottomspeed);
         secondaryConveyor.setSpeed(topspeed);
    }
 
 
    //DataCollection
 
    public int ballCount()
    {
        return primaryConveyor.ball + secondaryConveyor.ball;
    }
 
 
    //MUSTANGESUBSYSTEM
 
    @Override
    public HealthState checkHealth() {
        if(primaryConveyor.checkHealth()==HealthState.RED||secondaryConveyor.checkHealth()==HealthState.RED)
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
 
    public Conveyor(int id)
    {
       
        roller=SparkMAXFactory.buildSparkMAX(id, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
 
        conveyorSpeed =0;
 
        beamBreak = new BeamBreak(0);
 
        absConveyorSpeed = Math.abs(conveyorSpeed);
 
    }
 

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
 

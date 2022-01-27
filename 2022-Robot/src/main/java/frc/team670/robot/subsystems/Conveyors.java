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
    public Conveyor intakeConveyor, shooterConveyor;
 
 
    public Conveyors(){
        intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR);
        shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR);
 
    }
 
    // ACTIONS
 
    public void runConveyors(boolean intaking,boolean shooting)
    {
        
        intakeConveyor.run(intaking);
        if(shooting||!intaking)
        {
            shooterConveyor.run(intaking);
        }
        
    }
 
    public void stopAll()
    {
        intakeConveyor.stop();
        shooterConveyor.stop();
    }
 
 
    //DataCollection
 
    public int ballCount()
    {
        return intakeConveyor.ball + shooterConveyor.ball;
    }
 
 
    //MUSTANGESUBSYSTEM
 
    @Override
    public HealthState checkHealth() {
        if(intakeConveyor.checkHealth()==HealthState.RED||shooterConveyor.checkHealth()==HealthState.RED)
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
 
    private double conveyorSpeed=1.0;
 
 
    public int ball = 0;
   
 
    BeamBreak beamBreak;
 
    double absConveyorSpeed=1.0;
 
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
            
            return true;
        }
 
        ball = 0;
        
 
        return false;
       
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
 
    public void stop()
    {
        roller.stopMotor();;
    }
 
    public void setSpeed(double speed)
    {
        conveyorSpeed = speed;
        Logger.consoleLog("Speed: " + conveyorSpeed);
    }
 
 
    @Override
    public HealthState checkHealth()
    {
        REVLibError rollerError=roller.getLastError();
        if ( (rollerError != null) && (rollerError != REVLibError.kOk) ) {
            return HealthState.RED;
        }
        
       
       
        return HealthState.GREEN;
    }
 
    @Override
    public void mustangPeriodic()
    {
        running();
    }
 
}
 
 


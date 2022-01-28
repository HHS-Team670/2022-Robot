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
    private String status="OFF"; 
 
 
    public Conveyors(){
        intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR);
        shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR);
 
    }
 
    // ACTIONS
 
    public void runConveyors(boolean intaking,boolean shooting)
    {
        
        intakeConveyor.run(intaking||shooting);
        shooterConveyor.run(intaking||shooting);
        if(shooting||!intaking)
        {
            
            if(!intaking)
            {
                status = "OUTTAKING";
            }else 
            {
                status = "SHOOTING";
            }
        }else
        {
            status = "INTAKING";
        }
        
        
    }
    public void changeState()
    {
        if(status.equals("INTAKING"))
        {
            if(shooterConveyor.ballCount==1)
            {
                shooterConveyor.stop();
                if(intakeConveyor.ballCount==1)
                {
                    intakeConveyor.stop();
                }
            }           

        }else if(status.equals("SHOOTING")||status.equals("OUTTAKING")) 
        {
            if(ballCount()==0)
            {
                stopAll();
            }
        }


    }
 
    public void stopAll()
    {
        status="OFF";
        intakeConveyor.stop();
        shooterConveyor.stop();
    }
 
 
    //DataCollection
 
    public int ballCount()
    {
        return intakeConveyor.ballCount + shooterConveyor.ballCount;
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
        changeState();
       
    }
 
 
 
 
 
}
 
 
 
class Conveyor extends MustangSubsystemBase
{
 
    private SparkMAXLite roller;
 
    private double conveyorSpeed=1.0;
 
 
    public int ballCount = 0;
   
 
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
            ballCount = 1;
            
            return true;
        }
 
        ballCount = 0;
        
 
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
 
 


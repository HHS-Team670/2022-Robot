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
    //CONVEYOR STATUS
    public enum Status
    {
        OFF,INTAKING,OUTTAKING,SHOOTING
    }
    public Conveyor intakeConveyor, shooterConveyor;
    private Status status = Status.OFF; 
 
 
    public Conveyors(){
        intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR, RobotMap.INTAKE_CONVEYOR_BEAMBREAK); 
        shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR, RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);  
 
    }
 
    // ACTIONS

    //RUNS THE CONVEYOR IN THE GIVEN MODE
    public void runConveyor(Status mode)
    {
        if(mode == Status.INTAKING)
        {
            intakeConveyor();
        }else if (mode == Status.SHOOTING)
        {
            shootConveyor();
        }else if(mode == Status.OUTTAKING)
        {
            outtakeConveyor();
        }else if(mode == Status.OFF)
        {
            stopAll();
        }
    }
    //HELPER METHOF TO RUNCONVEYOR
    private void intakeConveyor () {
        intakeConveyor.run(true);
        shooterConveyor.run(true);
        status = Status.INTAKING;
        Logger.consoleLog("Conveyor Status: INTAKING");
        


    }
    //HELPER METHOF TO RUNCONVEYOR
    private void shootConveyor () {
        intakeConveyor.run(true);
        shooterConveyor.run(true);
        status = Status.SHOOTING;
        Logger.consoleLog("Conveyor Status: SHOOTING");
        
    }
    //HELPER METHOF TO RUNCONVEYOR
    private void outtakeConveyor()
    {
        intakeConveyor.run(false);
        shooterConveyor.run(false);
        status = Status.OUTTAKING;
        Logger.consoleLog("Conveyor Status: OUTTAKING");
    }
    //USES THE CURRENT STATE OF THE CONVEYOR TO DECIDE WHICH PARTS NEED TO BE SHUT DOWN
    private void changeState()
    {
        switch(status)
        {
            case INTAKING:
                if(shooterConveyor.getBallCount() == 1)
                {
                    shooterConveyor.stop();
                    if(intakeConveyor.getBallCount() == 1)
                    {
                        intakeConveyor.stop();
                    }
                }
                break;
            case OUTTAKING:
                if(shooterConveyor.getBallCount()==0)
                {
                    shooterConveyor.stop();
                }
                if(ballCount()==0)
                {
                    stopAll();
                }
                break;
            case SHOOTING:
                if(intakeConveyor.getBallCount()==0)
                {
                    intakeConveyor.stop();
                }
                if(ballCount()==0)
                {
                    stopAll();
                }
                break;
            case OFF:
                break;
        }
        


    }
    //STOPS THE CONVEYORS
    public void stopAll()
    {
        status = Status.OFF;
        intakeConveyor.stop();
        shooterConveyor.stop();
    }

    
    //DATA COLLECTION
 
    //RETURN THE TOTAL NUMBER OF BALLS IN THE CONVEYORS
    public int ballCount()
    {
        return intakeConveyor.getBallCount() + shooterConveyor.getBallCount();
    }
 
 
    //MUSTANGESUBSYSTEM
 
    @Override
    public HealthState checkHealth() {
        REVLibError intakeError=intakeConveyor.getRollerError();
        REVLibError shooterError=shooterConveyor.getRollerError();
        if((intakeError!=null&& intakeError!= REVLibError.kOk)||(shooterError!=null&& shooterError!= REVLibError.kOk))
        {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }
 
    @Override
    public void mustangPeriodic() {
        checkHealth();
        changeState();
        intakeConveyor.isFull();
        shooterConveyor.isFull();
    }
 
 
 
 
 
}
 
 
 
class Conveyor
{
 
    private SparkMAXLite roller;
 
    private double conveyorSpeed=1.0;
 
 
    private int ballCount = 0;
   
 
    BeamBreak beamBreak;
 
    
 
    public Conveyor(int motorID, int beamBreakID)
    {
       
        roller=SparkMAXFactory.buildSparkMAX(motorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
 
        conveyorSpeed =0;
 
        beamBreak = new BeamBreak(beamBreakID);
 
        
    }
    public int getBallCount()
    {
        
        return ballCount;
    }
 
 
    //RETURNS TRUE IF THE CONVEYOR IS FULL
    public boolean isFull()
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
    //RUNS THE CONVEYOR IN THE SPECIFIED DIRECTION
    public void run(boolean intaking)
    {
        if (!intaking)
        {
            roller.set(-1*conveyorSpeed);
        } else
        {
            roller.set(conveyorSpeed);
        }
       
        
    }

    //STOPS THE CONVEYOR
    public void stop()
    {
        roller.stopMotor();;
    }
 

    //RETURNS THE ERROR THE ROLLOR IS CURRENTLY GIVING
    public REVLibError getRollerError()
    {
        return roller.getLastError();
    }
 
 
   
    
 
    
 
}
 
 


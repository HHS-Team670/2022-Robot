package frc.team670.robot.subsystems;
 
import com.revrobotics.REVLibError;
 
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
 
//Conveyor Status
enum Status
{
    OFF,INTAKING,OUTTAKING,SHOOTING
}
public class Conveyors extends MustangSubsystemBase
{
    public Conveyor intakeConveyor, shooterConveyor;
    private Status status = Status.OFF; 
 
 
    public Conveyors(){
        intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR, 0); // 0 is the ID for the beamBreak. I doubt this will be final, so remember to change it
        shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR, 1);  // 1 is the ID for the beamBreak. I doubt this will be final, so remember to change it
 
    }
 
    // ACTIONS

    //Runs the conveyor the params are used to decide iuf you are intaking shooting or outtaking.
    public void runConveyor(boolean intaking, boolean shooting)
    {
        if(intaking)
        {
            intakeConveyor();
        }else if (shooting)
        {
            shootConveyor();
        }else 
        {
            outtakeConveyor();
        }
    }
    //Helper Method to runConveyor
    private void intakeConveyor () {
        intakeConveyor.run(true);
        shooterConveyor.run(true);
        status = Status.INTAKING;
        Logger.consoleLog("Conveyor Status: INTAKING");
        


    }
    //Helper method to runConveyor
    private void shootConveyor () {
        intakeConveyor.run(true);
        shooterConveyor.run(true);
        status = Status.SHOOTING;
        Logger.consoleLog("Conveyor States: SHOOTING");
        
    }
    //Helper method to runConveyor
    private void outtakeConveyor()
    {
        intakeConveyor.run(false);
        shooterConveyor.run(false);
        status = Status.OUTTAKING;
        Logger.consoleLog("Conveyor States: OUTTAKING");
    }
    //Uses the current state of the Conveyor to decide what parts of the conveyor should shut down
    private void changeState()
    {
        switch(status)
        {
            case INTAKING:
                if(shooterConveyor.ballCount == 1)
                {
                    shooterConveyor.stop();
                    if(intakeConveyor.ballCount == 1)
                    {
                        intakeConveyor.stop();
                    }
                }
                break;
            case OUTTAKING:
                if(shooterConveyor.ballCount==0)
                {
                    shooterConveyor.stop();
                }
                if(ballCount()==0)
                {
                    stopAll();
                }
                break;
            case SHOOTING:
                if(intakeConveyor.ballCount==0)
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
    //Stops the conveyors
    public void stopAll()
    {
        status = Status.OFF;
        intakeConveyor.stop();
        shooterConveyor.stop();
    }

    //Sets the speed for the conveyors
    public void setSpeed(double intakeConveyorSpeed, double shooterConveyorSpeed) {
        intakeConveyor.setSpeed(intakeConveyorSpeed);
        shooterConveyor.setSpeed(shooterConveyorSpeed);
    }
    //DataCollection
 
    //Returns the total number of balls in the conveyor subsystem
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
 
    public Conveyor(int motorID, int beamBreakID)
    {
       
        roller=SparkMAXFactory.buildSparkMAX(motorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
 
        conveyorSpeed =0;
 
        beamBreak = new BeamBreak(beamBreakID);
 
        absConveyorSpeed = Math.abs(conveyorSpeed);
 
    }
 
 
    //returns if the conveyor is full
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
    // runs the conveyor in the direction specified by the parameter
    public void run(boolean notOuttaking)
    {
        if (!notOuttaking)
        {
            conveyorSpeed = absConveyorSpeed * -1;
        } else
        {
            conveyorSpeed = absConveyorSpeed;
        }
       
        roller.set(conveyorSpeed);
    }

    //Stops the conveyor
    public void stop()
    {
        roller.stopMotor();;
    }
 
    //Cahanges the speed of the conveyor
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
        isFull();
    }
 
}
 
 


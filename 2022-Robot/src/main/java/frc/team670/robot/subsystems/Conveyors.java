package frc.team670.robot.subsystems;
 
import com.revrobotics.REVLibError;
 
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
 
/*
Note: Conveyor states are represented by the following numbers
Off - 0
Intaking - 1
Outtaking - 2
Shooting - 3
*/
public class Conveyors extends MustangSubsystemBase
{
    public Conveyor intakeConveyor, shooterConveyor;
    private int status = 0; 
 
 
    public Conveyors(){
        intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR, 0); // 0 is the ID for the beamBreak. I doubt this will be final, so remember to change it
        shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR, 1);  // 1 is the ID for the beamBreak. I doubt this will be final, so remember to change it
 
    }
 
    // ACTIONS

    public void runIntakeConveyor (boolean intaking) {
        intakeConveyor.run(intaking);
        if (intaking) {
            status = 1;
        }

        else {
            status = 2;
        }
    }

    public void runShooterConveyor (boolean shooting) {
        shooterConveyor.run(shooting);
        if (shooting) {
            status = 3;
        }
    }
    private void changeState()
    {
        if(status == 1)
        {
            if(shooterConveyor.ballCount == 1)
            {
                shooterConveyor.stop();
                if(intakeConveyor.ballCount == 1)
                {
                    intakeConveyor.stop();
                }
            }           

        }else if(status == (3)||status == (2)) 
        {
            if(ballCount() == 0)
            {
                stopAll();
            }
        }


    }
 
    public void stopAll()
    {
        status = 0;
        intakeConveyor.stop();
        shooterConveyor.stop();
    }

    public void setSpeed(double intakeConveyorSpeed, double shooterConveyorSpeed) {
        intakeConveyor.setSpeed(intakeConveyorSpeed);
        shooterConveyor.setSpeed(shooterConveyorSpeed);
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
 
    public Conveyor(int motorID, int beamBreakID)
    {
       
        roller=SparkMAXFactory.buildSparkMAX(motorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
 
        conveyorSpeed =0;
 
        beamBreak = new BeamBreak(beamBreakID);
 
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
 
 


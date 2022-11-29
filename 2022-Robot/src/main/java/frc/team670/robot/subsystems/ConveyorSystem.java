package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/**
 * Connects the intake to the shooter
 * 
 * @author Armaan, Soham, Edward
 */
public class ConveyorSystem extends MustangSubsystemBase {

    SparkMAXLite conveyor1Motor;
    SparkMAXLite conveyor2Motor;
    BeamBreak bb1;
    BeamBreak bb2;
    int balls;
    String status;
    

    public ConveyorSystem() {
        conveyor1Motor = SparkMAXFactory.buildSparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig,
            Motor_Type.NEO_550);
        conveyor2Motor = SparkMAXFactory.buildSparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig,
            Motor_Type.NEO_550);
        bb1 = new BeamBreak(RobotMap.INTAKE_CONVEYOR_MOTOR);
        bb2 = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_MOTOR);
        //Independently control each motor
        //Prevent balls from accidently firing
        //When the shooter beam break is triggered we want the cshooter conveyor to stop
        //When the intale beam break is triggered and the shooter conveyor is full the intake should stop
        //Methods: stopAll(), isRunning(),
        
    }
        //both conveyors are on initially
    public void setIntakeConveyor(double speed){
        conveyor1Motor.set(speed);
    }
    public void setShooterConveyor(double speed){
        conveyor2Motor.set(speed);
    }
    public void check(){
    //    if(status.equals("INTAKING"))
        if(bb2.isTriggered()){
            conveyor2Motor.set(0);
            if(bb1.isTriggered()){
            conveyor1Motor.set(0);}
            }
        //if status == shooting 
            //if getballcount==0
    }

    public void stopAll(){
        conveyor1Motor.set(0);
        conveyor2Motor.set(0);

    }
    //Conveyor has 3 modes or statuses
    // u have code for intaking 
    // there is also shooting and ejecting
    // for shooting run both conveyors at 0.7 until getballcount is 0
    // for ejecting run both conveyors at -0.7 until getballcount is 0
    // conveyors.startCoveyors("Intaking")
    // automatically start intake
    // store status so that u can do stopping conditions

public void startConveyor(String status){
    this.status=status;
    if(status.equals("INTAKING")){
        
    }
    if(status.equals("SHOOTING")){
        modeS();
        getBallCount();
    }
    if(status.equals("EJECTING")){
        getBallCount();
        // if(getBallCount()==0)
    }

}
    
public void modeS(){
    setIntakeConveyor(0.7);
    setShooterConveyor(0.7);
}

    public boolean isRunning(){
        // conveyor1Motor.set(double speed);
        // conveyor2Motor.set(double speed);
        if(conveyor1Motor.get()!=0||conveyor2Motor.get()!=0){
            return true;

        }else{
            return false;
        }
        
    }
    public int getBallCount(){
        if(bb1.isTriggered() && bb2.isTriggered()){
            return 2;
        }
        //if bb1 is triggered and && bb2 is triggered then return 2 if either is triggered return 1 if neither are triggered return 0
        if(bb1.isTriggered()||bb2.isTriggered()){
            return 1;
        }
        if(!bb1.isTriggered()&&!bb2.isTriggered()){
                return 0;
        }
        return -1;
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return HealthState.GREEN;
    }
    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
        check();
        
    }
    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
    }
}
    
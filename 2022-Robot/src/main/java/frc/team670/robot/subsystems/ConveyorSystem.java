package frc.team670.robot.subsystems;

import org.ejml.ops.ConvertMatrixData;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/**\
 * 
 * Connects the intake to the shooter
 * anyone know how to use Enums
 * @author Armaan, Soham, Edward
 */

public class ConveyorSystem extends MustangSubsystemBase{
    public static enum Status { 
        INTAKING,
        SHOOTING,
        EJECTING,
        OFF
    }
    SparkMAXLite Conveyor1Motor;
    SparkMAXLite Conveyor2Motor;
    BeamBreak bb1;
    BeamBreak bb2;
    int ballcount;
    Status status;
    public void Conveyor(){
        Conveyor1Motor=SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        Conveyor2Motor=SparkMAXFactory.buildFactorySparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        this.bb1=new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        boolean t= bb1.isTriggered();
        this.bb2=new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
        this.ballcount=0;
        this.status=Status.INTAKING;
        
    }
    public void SetC1(double speed) {
        Conveyor1Motor.set(speed);



    }
    public void SetC2(double speed) {
        Conveyor2Motor.set(speed);


    }
    @Override
    public HealthState checkHealth(){
        return HealthState.GREEN;
    }
    @Override
    public void mustangPeriodic(){
        getBallCount();
        motors(this.status);

    }
    public void motors(ConveyorSystem.Status Status){
        this.status=Status;
        if (status==ConveyorSystem.Status.INTAKING){
            if (ballcount==0||(ballcount==1 && bb2.isTriggered())){
                Conveyor1Motor.set(0.7);
                Conveyor2Motor.set(0);
            }
            else if (ballcount==1 && bb1.isTriggered()){
                Conveyor2Motor.set(0.7);
                Conveyor1Motor.set(0.7);
            }
            else if (ballcount==2){
                Conveyor1Motor.set(0);
                Conveyor2Motor.set(0);

            }
        else if (status==ConveyorSystem.Status.SHOOTING){
             if (ballcount==2||(ballcount==1 && bb1.isTriggered())){
                 Conveyor2Motor.set(0.7);
                 Conveyor1Motor.set(0.7);
             }
             else if (ballcount==1 && bb2.isTriggered()){
                Conveyor2Motor.set(0.7);
            }
            else{
                return;
            }
        }
        else if (status==ConveyorSystem.Status.EJECTING){
            Conveyor1Motor.set(-1);
            Conveyor2Motor.set(-1);
        }
    } 
} 
        public Status getStatus(){
            return status;


        }


    public void stopAll(){
        Conveyor1Motor.set(0);
        Conveyor2Motor.set(0);
    }

 

        
    public int getBallCount(){
        Logger.consoleLog(""+bb2);
        boolean x=bb1.isTriggered();
        boolean y=bb2.isTriggered();
        if (x && y){
            ballcount=2;
            

        }
        else if ((bb1.isTriggered()==true && bb2.isTriggered()==false) || (bb1.isTriggered()==false && bb2.isTriggered()==true)){
            ballcount=1;
        }
        else if (bb1.isTriggered()==false && bb2.isTriggered()==false){
            ballcount=0;
        }
        return ballcount;
    }
    public boolean Status(){
        return (Conveyor1Motor.get()!=0)&&(Conveyor2Motor.get()!=0);

    }
    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
    }
    public boolean isRunning(){
        return (status!=ConveyorSystem.Status.OFF);

    }
}


   
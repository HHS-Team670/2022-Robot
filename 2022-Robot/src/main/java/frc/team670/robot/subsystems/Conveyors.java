package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.SpeedController;
import java.lang.AutoCloseable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import frc.team670.robot.constants.RobotMap;

public class Conveyors extends MustangSubsystemBase 
{

    private SparkMAXLite roller;
    private double conveyorSpeed;

    private boolean conveyorStates[] = {false, false, false};


    BeamBreak beamBreak1;
    BeamBreak beamBreak2;
    BeamBreak beamBreak3;
    public Conveyors(int id, MotorConfig.Motor_Type type, double speed) 
    {
        roller = new SparkMAXLite(id, type);
        conveyorSpeed = speed;
        //Check the dio port for the beamBreak sensors
        beamBreak1 = new BeamBreak(0);
        beamBreak2 = new BeamBreak(1);
        beamBreak3 = new BeamBreak(2);
        numBalls = 0;
    }

    public void run(boolean reversed) 
    {
        if (reversed) 
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

    public void updateConveyorStates () {
        for (int i = 0; i < conveyorStates.length)
    }

    public boolean isConveyorFull() {
        boolean conveyorCondition = false;
        if (beamBreak3.isTriggered() == true)
            conveyorCondition = true;
        
        return conveyorCondition;
    }

    public void useConveyor () {
        if (!isConveyorFull()) {
            run(false);
        }
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

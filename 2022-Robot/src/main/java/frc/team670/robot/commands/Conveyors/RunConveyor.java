package frc.team670.robot.commands.Conveyors;
import java.util.HashMap;
import java.util.Map;

import frc.team670.mustanglib.utils.Logger;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyors;

public class RunConveyor extends InstantCommand implements MustangCommand 
{

    private Conveyors conveyors;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean intaking;
/*
    public RunConveyor(Conveyors conveyors, boolean intaking)
    {
        this.conveyors = conveyors;
        addRequirements(conveyors);
        healthReqs = new HashMap < MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyors, HealthState.YELLOW);
        this.intaking = intaking;
    }

    public void initialize() {
        Logger.consoleLog("Running Conveyors");    
    }

    public void execute() {
        conveyors.runConveyors(intaking);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() 
    { 
        return healthReqs;
    }
}
*/
    public void execute() {
        conveyors.runConveyors(intaking);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.consoleLog("Indexer system emptied");
        conveyors.stop();
    }

    @Override
    public boolean isFinished() {
        if(indexer.getTotalNumBalls() == 0 && microSecondsSinceZeroBalls == -1){
            microSecondsSinceZeroBalls =  RobotController.getFPGATime();
        }
        if(microSecondsSinceZeroBalls != -1 && RobotController.getFPGATime() - microSecondsSinceZeroBalls >= 500000){
            microSecondsSinceZeroBalls = -1;
            return true;
        }
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
package frc.team670.robot.commands.Conveyors;

import java.util.HashMap;
import java.util.Map;

import frc.team670.mustanglib.utils.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyors;

public class RunConveyor extends CommandBase implements MustangCommand {

    private Conveyors conveyors;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean intaking;

    public RunConveyor(Conveyors conveyors,boolean intaking)
    {
        this.conveyors=conveyors;
        addRequirements(conveyors);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyors, HealthState.YELLOW);
        this.intaking=intaking;

        
    }

    public void initialize() {
        Logger.consoleLog("Running Conveyors");

    }

    public void execute() {
        conveyors.runConveyors(intaking);

    }
    public boolean isFinished()
    {
        if(conveyors.isEmpty())
        {
            return true;
        }
        return false;
    }


    public void end() {
        conveyors.stopAll();
    }
    

    

    
}

package frc.team670.robot.commands.Conveyors;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyors;

public class StopConveyor extends InstantCommand implements MustangCommand
{
    private Conveyors conveyors;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public StopConveyor(Conveyors conveyors)
    {
        this.conveyors=conveyors;
        addRequirements(conveyors);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();

		healthReqs.put(conveyors, HealthState.YELLOW);
    }

    public void initialize()
    {
        conveyors.stopAll();
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() 
    {
        return healthReqs;
    }    
    
}

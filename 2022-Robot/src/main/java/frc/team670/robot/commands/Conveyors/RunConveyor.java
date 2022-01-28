package frc.team670.robot.commands.Conveyors;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyors;

public class RunConveyor extends InstantCommand implements MustangCommand 
{

    private Conveyors conveyors;
    
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean intaking, shooting;
    



    public RunConveyor(Conveyors conveyors, boolean intaking, boolean shooting)
    {
        this.conveyors = conveyors;
        addRequirements(conveyors);
        healthReqs = new HashMap < MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyors, HealthState.YELLOW);
        this.intaking = intaking;
        this.shooting = shooting;

    }
    
    public void initialize()
    {
        
        conveyors.runConveyors(intaking,shooting);
        
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
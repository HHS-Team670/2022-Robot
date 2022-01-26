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
    private boolean intaking;
    private double c1Speed,c2Speed;



    public RunConveyor(Conveyors conveyors, boolean intaking,double c1Speed,double c2Speed)
    {
        this.conveyors = conveyors;
        addRequirements(conveyors);
        healthReqs = new HashMap < MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyors, HealthState.YELLOW);
        this.intaking = intaking;
        this.c1Speed = c1Speed;
        this.c2Speed = c2Speed;
    }
    
    public void initialize()
    {
        conveyors.setSpeed(c1Speed, c2Speed);
        conveyors.runConveyors(intaking);
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
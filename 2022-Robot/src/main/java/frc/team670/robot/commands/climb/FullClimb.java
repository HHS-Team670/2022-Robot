package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Once the driver aligns the back wheels to the bars under the hangar 
 * and drives straight to align with the climbing bar, this climbs on to the high bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
    
    private Climber climber;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public FullClimb(Climber c) {
        this.climber = c;
        addRequirements(c);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        addCommands(
                new ExtendClimber(climber, true),
                new HookOnBar(climber, true),
                new ExtendClimber(climber, false),
                new HookOnBar(climber, false),
                new UnhookFromBar(climber), // TODO: Do we really need to unhook??? And if we do should it come automatically with the hooking on to the higher bar???
                new RetractClimber(climber)
            );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}
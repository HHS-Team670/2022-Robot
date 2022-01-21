package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
    
    private Climber climber;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public FullClimb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            addCommands(
                new ExtendClimberStd(climber),
                new HookOnBar(climber),
                new ExtendClimberOblique(climber),
                new HookOnBarOblique(climber),
                new UnhookFromBar(climber), // Do we really need to unhook??? And if we do should it come automatically with the hooking on to the higher bar???
                new GetClimberDown(climber)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}
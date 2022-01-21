package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class UnhookFromBar extends CommandBase implements MustangCommand {

    private Climber climber;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean hooked;
    private boolean hooked2;

    public UnhookFromBar(Climber climber) {
        this.climber = climber;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        hooked = true;
        hooked2 = true;
    }

    @Override
    public void initialize() {
        hooked = true;
        hooked2 = true;
        climber.setPower1(0);
        climber.setPower2(0);
        climber.setPowerOblique1(0);
        climber.setPowerOblique2(0);
    }

    @Override
    public void execute() {
        if (hooked || hooked2) {
            climber.unhookFromBar();
        }
    }

    @Override
    public boolean isFinished() {
        return !climber.isHookedOnBar() && !climber.isHookedOnBar2();
    }

    @Override
    public void end(boolean interrupted){
        this.hooked = false;
        this.hooked2 = false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}

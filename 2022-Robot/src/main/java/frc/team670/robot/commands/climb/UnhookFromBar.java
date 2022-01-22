package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class UnhookFromBar extends ClimberBaseCommand {

    public UnhookFromBar(Climber climber) {
        super(climber);
    }

    @Override
    public void initialize() {
        climber.straight.unhookFromBar();
    }

    @Override
    public boolean isFinished() {
        return !climber.straight.isHookedOnBar();
    }
}

package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class HookOnBarStd extends ClimberBaseCommand {

    public HookOnBarStd(Climber climber) {
        super(climber);
    }

    @Override
    public void initialize() {
        climber.straight.hookOnBar();
    }

    @Override
    public boolean isFinished() {
        return climber.straight.isHookedOnBar();
    }
}
package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class HookOnBarOblique extends ClimberBaseCommand {

    public HookOnBarOblique(Climber climber) {
        super(climber);
    }

    @Override
    public void initialize() {
        climber.oblique.hookOnBar();
    }

    @Override
    public boolean isFinished() {
        return climber.oblique.isHookedOnBar();
    }
}
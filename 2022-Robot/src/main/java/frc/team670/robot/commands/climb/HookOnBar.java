package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class HookOnBar extends ClimberBaseCommand {
    
    private boolean straight;

    public HookOnBar(Climber climber, boolean str) {
        super(climber);
        straight = str;
    }

    @Override
    public void initialize() {
        if (straight)
        {
            climber.straight.hookOnBar();
        }
        else
        {
            climber.oblique.hookOnBar();
        }
    }

    @Override
    public boolean isFinished() {
        if (straight)
        {
            return climber.straight.isHookedOnBar();
        }
        else
        {
            return climber.oblique.isHookedOnBar();
        }
    }
}
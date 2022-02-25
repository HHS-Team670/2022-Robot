package frc.team670.robot.commands.climber;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class HookOnBar extends ClimberBaseCommand {

  public HookOnBar(Climber climber, boolean str) {
    super(climber, str);
  }

  @Override
  public void initialize() {
    telescopingClimber.hookOnBar();
  }

  @Override
  public boolean isFinished() {
    return telescopingClimber.isHookedOnBar();
  }
}
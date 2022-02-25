package frc.team670.robot.commands.climber;

import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class UnhookFromBar extends ClimberBaseCommand {

  public UnhookFromBar(Climber climber) {
    super(climber, true);
  }

  @Override
  public void initialize() {
    telescopingClimber.unhookFromBar();
  }

  @Override
  public boolean isFinished() {
    return !telescopingClimber.isHookedOnBar();
  }
}

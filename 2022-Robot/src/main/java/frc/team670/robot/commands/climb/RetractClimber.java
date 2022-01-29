package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Lower the Climber mechanism.
 */
public class RetractClimber extends ClimberBaseCommand {

  public RetractClimber(Climber climber) {
    super(climber, true);
  }

  @Override
  public void initialize() {
    super.initialize();
    telescopingClimber.climb(0);
  }


  @Override
  public boolean isFinished() {
    return telescopingClimber.isAtTarget();
  }
}

package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class ExtendClimber extends ClimberBaseCommand {
  

  public ExtendClimber(Climber climber, boolean straight) {
    super(climber, straight);
  }


  @Override
  public void initialize() {
    super.initialize();
    telescopingClimber.climb(telescopingClimber.MAX_EXTENDING_HEIGHT_CM);
  }

  @Override
  public boolean isFinished() {
    return telescopingClimber.isAtTarget();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import frc.team670.robot.subsystems.Climber;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class GetClimberDown extends ClimberBaseCommand {

  public GetClimberDown(Climber climber)
  {
    super(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    climber.straight.climb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.oblique.isAtTarget();
  }
}

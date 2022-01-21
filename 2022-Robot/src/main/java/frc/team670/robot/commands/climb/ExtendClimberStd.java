/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class ExtendClimberStd extends CommandBase implements MustangCommand {

  private Climber climber;
  private static final double MAX_EXTENDING_HEIGHT_CM = 66.24;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public ExtendClimberStd(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPower1(0);
    climber.setPower2(0);
    climber.setPowerOblique1(0);
    climber.setPowerOblique2(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climb(MAX_EXTENDING_HEIGHT_CM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setPower1(0);
    climber.setPower2(0);
    climber.setPowerOblique1(0);
    climber.setPowerOblique2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtTarget() && climber.isAtTarget2();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}

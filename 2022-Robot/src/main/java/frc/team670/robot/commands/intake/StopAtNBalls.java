package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;

/**
 * Runs the conveyor in the given mode
 * 
 * @author Armaan
 * @author Soham
 * @author Nancy
 * @author Scintilla
 * 
 */
public class StopAtNBalls extends CommandBase implements MustangCommand {

  private ConveyorSystem conveyors;
  private int nBalls;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public StopAtNBalls(ConveyorSystem conveyors, int balls) {
    this.conveyors = conveyors;
    nBalls = balls;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyors, HealthState.GREEN);
  }

  public boolean isFinished() {
    return (conveyors.ballCount() == nBalls);
  }


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}

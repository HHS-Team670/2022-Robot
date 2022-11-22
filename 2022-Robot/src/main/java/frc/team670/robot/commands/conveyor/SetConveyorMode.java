package frc.team670.robot.commands.conveyor;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class SetConveyorMode extends InstantCommand implements MustangCommand {

  private ConveyorSystem conveyors;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public SetConveyorMode(ConveyorSystem conveyor, String Status) {
    this.conveyors = conveyors;
    addRequirements(conveyors);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyors, HealthState.GREEN);
    conveyor.setStatus(Status);
  }

  public void initialize() {
    conveyors.setC1(1);
    conveyors.setC2(1);
  }


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}

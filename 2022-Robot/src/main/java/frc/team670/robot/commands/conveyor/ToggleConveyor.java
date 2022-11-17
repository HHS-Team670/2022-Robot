package frc.team670.robot.commands.conveyor;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;


/**
 * If the conveyor is intaking, it turns it off.
 * If the conveyor is off or running in another mode, it switches to Intaking mode.
 */
public class ToggleConveyor extends InstantCommand implements MustangCommand {

    private ConveyorSystem conveyors;

    public ToggleConveyor(ConveyorSystem conveyors) {
        this.conveyors = conveyors;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        HashMap<MustangSubsystemBase, HealthState> requirements = new HashMap<>();
        requirements.put(conveyors, HealthState.RED);
        return requirements;
    }

    @Override
    public void initialize() {
        if(conveyors.getStatus() == Status.INTAKING) {
            conveyors.stopAll();
        } else {
            conveyors.setConveyorMode(Status.INTAKING);
        }
    }
    
}

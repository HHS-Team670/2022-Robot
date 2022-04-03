package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;

public class ToggleColorSensorOverride extends InstantCommand implements MustangCommand {

    private ConveyorSystem conveyorSystem;

    public ToggleColorSensorOverride(ConveyorSystem conveyorSystem) {
        this.conveyorSystem = conveyorSystem;
    }

    @Override
    public void execute() {
        conveyorSystem.toggleColorSensorOverride();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<>();
    }
    
}

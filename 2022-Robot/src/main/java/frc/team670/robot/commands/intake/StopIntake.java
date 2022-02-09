package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

/**
 * Stops the intake
 * 
 * @author Sanatan
 * @author Armaan
 */
public class StopIntake extends InstantCommand implements MustangCommand {

    Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private ConveyorSystem conveyor;

    // Prepares everything to stop the intake

    public StopIntake(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.YELLOW);
        addRequirements(intake);
        this.conveyor = conveyor;
    }

    // Stops the intake
    public void initialize() {
        intake.stop();
        conveyor.stopAll();
    }

    // Returns the health state
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
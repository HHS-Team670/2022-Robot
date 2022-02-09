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
 * Stops and retracts the intake
 * 
 * @author Sanatan
 * @author Armaan
 */
public class StopAndRetractIntake extends CommandBase implements MustangCommand {

    Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private ConveyorSystem conveyor;

    // Prepares everything to stop the intake

    public StopAndRetractIntake(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.YELLOW);
        addRequirements(intake);
        this.conveyor = conveyor;
    }

    // Stops the intake

    public void initialize() {
        intake.stop();
        intake.deployer.retractIntake();
        conveyor.stopAll();
    }

    public boolean isFinished() {
        return !intake.deployer.isAtTarget();
    }

    // Returns the health state

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
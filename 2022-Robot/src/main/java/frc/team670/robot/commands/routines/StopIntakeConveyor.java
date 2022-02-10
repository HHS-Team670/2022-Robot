package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.conveyor.*;
import frc.team670.robot.commands.intake.StopIntake;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * Stops the intake and the conveyor
 * @author Sanatan, Armaan
 */


public class StopIntakeConveyor extends ParallelCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public StopIntakeConveyor(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(this.intake);
        addRequirements(this.conveyor);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(this.intake, HealthState.YELLOW);
        healthReqs.put(this.conveyor, HealthState.GREEN);
        addCommands(
            new StopIntake(intake, conveyor),
            new StopConveyor(conveyor));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}

package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.conveyor.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * Empties intake and conveyor
 * @author Sanatan
 */

 
public class EmptyIntakeConveyor extends SequentialCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public EmptyIntakeConveyor(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(this.intake);
        addRequirements(this.conveyor);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(this.intake, HealthState.YELLOW);
        healthReqs.put(this.conveyor, HealthState.GREEN);
        addCommands(
            new ParallelCommandGroup(
                new RunIntake(true, this.intake),
                new RunConveyor(this.conveyor, ConveyorSystem.Status.OUTTAKING)),
            new StopAtNBalls(this.conveyor, 0),
            new StopIntake(this.intake));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}

<<<<<<< HEAD:2022-Robot/src/main/java/frc/team670/robot/commands/routines/StopIntakeConveyor.java
package frc.team670.robot.commands.routines;
=======
package frc.team670.robot.commands.routines.intake;
>>>>>>> dev:2022-Robot/src/main/java/frc/team670/robot/commands/routines/intake/EmptyRobot.java

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
<<<<<<< HEAD:2022-Robot/src/main/java/frc/team670/robot/commands/routines/StopIntakeConveyor.java
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.conveyor.*;
import frc.team670.robot.commands.intake.StopIntake;
import edu.wpi.first.wpilibj2.command.WaitCommand;
=======
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
>>>>>>> dev:2022-Robot/src/main/java/frc/team670/robot/commands/routines/intake/EmptyRobot.java


/**
 * Stops the intake and the conveyor
 * @author Sanatan, Armaan
 */

 
public class EmptyRobot extends ParallelCommandGroup implements MustangCommand {

<<<<<<< HEAD:2022-Robot/src/main/java/frc/team670/robot/commands/routines/StopIntakeConveyor.java
public class StopIntakeConveyor extends ParallelCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
=======
>>>>>>> dev:2022-Robot/src/main/java/frc/team670/robot/commands/routines/intake/EmptyRobot.java
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

<<<<<<< HEAD:2022-Robot/src/main/java/frc/team670/robot/commands/routines/StopIntakeConveyor.java
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
=======
    public EmptyRobot(Intake intake, ConveyorSystem conveyor) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        addCommands(
            new RunIntake(intake, true),
            new RunConveyor(conveyor, ConveyorSystem.Status.OUTTAKING));
>>>>>>> dev:2022-Robot/src/main/java/frc/team670/robot/commands/routines/intake/EmptyRobot.java
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}

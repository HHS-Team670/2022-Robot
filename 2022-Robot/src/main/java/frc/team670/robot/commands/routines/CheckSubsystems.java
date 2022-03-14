package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.ClimberSystem.Level;
import frc.team670.robot.subsystems.ConveyorSystem.Status;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Checks function for all subsystems by running through commands for all of
 * them
 */
public class CheckSubsystems extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private Climber verticalClimber, diagonalClimber;
    private Intake intake;
    private Deployer deployer;
    private ConveyorSystem conveyors;
    private Shooter shooter;

    private MustangController controller;

    private int currentStep = 0;
    private boolean justAdvanced = false;

    public CheckSubsystems(Intake intake, Deployer deployer, ConveyorSystem conveyors, Shooter shooter,
            ClimberSystem climbers, MustangController controller) {
        verticalClimber = climbers.getVerticalClimber();
        diagonalClimber = climbers.getDiagonalClimber();
        addRequirements(verticalClimber, diagonalClimber, climbers);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(deployer, HealthState.GREEN);
        healthReqs.put(conveyors, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(verticalClimber, HealthState.GREEN);
        healthReqs.put(diagonalClimber, HealthState.GREEN);
        healthReqs.put(climbers, HealthState.GREEN);        
        this.intake = intake;
        this.deployer = deployer;
        this.conveyors = conveyors;
        this.shooter = shooter;
        this.controller = controller;
    }

    // Called once when the command executes
    @Override
    public void execute() {
        if (!justAdvanced) {
            if (controller.getDPadState() == MustangController.DPadState.RIGHT) {
                testProcedure(++currentStep);
                justAdvanced = true;
            } else if (controller.getDPadState() == MustangController.DPadState.LEFT) {
                testProcedure(--currentStep);
                justAdvanced = true;
            }
        } else if (controller.getDPadState() == MustangController.DPadState.NEUTRAL) {
            justAdvanced = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    public void testProcedure(int step) {
        switch (step) {
            case 0:
                deployer.deploy(false);
                intake.stop();
                break;
            case 1:
                deployer.deploy(true);
                intake.roll(false);
                break;
            case 2:
                conveyors.runConveyor(Status.INTAKING);
                break;
            case 3:
                conveyors.runConveyor(Status.OUTTAKING);
                break;
            case 4:
                shooter.setTargetRPM(shooter.getDefaultRPM());
                shooter.run();
                break;
            case 5:
                shooter.stop();
                break;
            case 6:
                verticalClimber.retract();
                diagonalClimber.retract();
                break;
            case 7:
                verticalClimber.climb(Level.MID);
                diagonalClimber.retract();
                break;
            case 8:
                verticalClimber.retract();
                diagonalClimber.retract();
                break;
            case 9:
                diagonalClimber.climb(Level.HIGH);
                verticalClimber.retract();
                break;
            case 10:
                verticalClimber.climb(Level.INTERMEDIATE_MID);
                diagonalClimber.climb(Level.HIGH);
                break;
            case 11:
                verticalClimber.retract();
                diagonalClimber.climb(Level.HIGH);
                break;
            default:
                break;
        }
    }

}
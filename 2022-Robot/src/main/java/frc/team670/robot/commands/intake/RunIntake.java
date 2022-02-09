package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Intake;

/**
 * Runs the intake
 */
public class RunIntake extends InstantCommand implements MustangCommand {

    Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;
    private Intake intake;

    /**
     * @param reversed true to run the intake in reverse (out), 
     * false to run it normally (in)
     */
    public RunIntake(boolean reversed, Intake intake) {
        this.reversed = reversed;
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.YELLOW);
    }
    
    /*
    starts to roll the intake
    */
    public void initialize() {
        intake.roll(reversed);
    }

    /* 
    returns the health state
    */
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
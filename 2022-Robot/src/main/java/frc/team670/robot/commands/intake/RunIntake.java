package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Intake;

/**
 * Runs the intake, controlls the direction based on wheter or not it is jammed
 */
public class RunIntake extends CommandBase implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private boolean reversed;
	private Intake intake;
	private int countWasJammed;

	/**
	 * @param reversed true to run the intake in reverse (out), 
	 * false to run it normally (in)
	 */
	public RunIntake(boolean reversed, Intake intake) {
		this.reversed = reversed;
		this.intake = intake;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
		addRequirements(intake);
		countWasJammed = 0;
	}

	public void initialize() {
		intake.roll(reversed);
	}

	public void execute() {
		SmartDashboard.putNumber("count jammed", countWasJammed);
		if (intake.isJammed()) {
			countWasJammed = 150;
		}
		if (countWasJammed > 0) {
			intake.roll(!reversed);
			countWasJammed--;
		} else {
			intake.roll(reversed);
		}
	}

	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}
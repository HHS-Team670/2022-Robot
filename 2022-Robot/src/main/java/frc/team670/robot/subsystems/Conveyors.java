
package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.Logger;
/**
 * Connects the intake to the shooter
 * 
 * @author Armaan
 * @author Soham
 * @author Edward
 */
public class Conveyors extends MustangSubsystemBase {
	// Conveyor status
	public enum Status {
		OFF,
		INTAKING,
		OUTTAKING,
		SHOOTING
	}

	private Conveyor intakeConveyor, shooterConveyor;
	private Status status = Status.OFF;

	public Conveyors() {
		intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR, RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
		shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR, RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
	}
	// Actions
	// Runs the Conveyor in the given mode
	public void runConveyor(Status mode) {
		if (mode == Status.INTAKING) {
			intakeConveyor();
		} else if (mode == Status.SHOOTING) {
			shootConveyor();
		} else if (mode == Status.OUTTAKING) {
			outtakeConveyor();
		} else if (mode == Status.OFF) {
			stopAll();
		}
	}
	// Helper method of runconveyor
	private void intakeConveyor() {
		intakeConveyor.run(true);
		shooterConveyor.run(true);
		status = Status.INTAKING;
		Logger.consoleLog("Conveyor Status: INTAKING");
	}
	// Helper method of runconveyor
	private void shootConveyor() {
		intakeConveyor.run(true);
		shooterConveyor.run(true);
		status = Status.SHOOTING;
		Logger.consoleLog("Conveyor Status: SHOOTING");
	}
	// Helper method of runconveyor
	private void outtakeConveyor() {
		intakeConveyor.run(false);
		shooterConveyor.run(false);
		status = Status.OUTTAKING;
		Logger.consoleLog("Conveyor Status: OUTTAKING");
	}
	// Uses the current state of the conveyor to determine what parts need to be shut down
	private void checkState() {
		switch (status) {
			case INTAKING:
				if (shooterConveyor.getBallCount() == 1) {
					shooterConveyor.stop();
					if (intakeConveyor.getBallCount() == 1) {
						intakeConveyor.stop();
					}
				}
				break;
			case OUTTAKING:
				if (shooterConveyor.getBallCount() == 0) {
					shooterConveyor.stop();
				}
				if (ballCount() == 0) {
					intakeConveyor.stop();
				}
				break;
			case SHOOTING:
				if (intakeConveyor.getBallCount() == 0) {
					intakeConveyor.stop();
				}
				if (ballCount() == 0) {
					shooterConveyor.stop();
				}
				break;
			case OFF:
				break;
		}
	}
	// Stops the conveyors
	public void stopAll() {
		status = Status.OFF;
		intakeConveyor.stop();
		shooterConveyor.stop();
	}
	// Data collection
	// Returns the total number of balls in the conveyor
	public int ballCount() {
		return intakeConveyor.getBallCount() + shooterConveyor.getBallCount();
	}
	// Mustang Subsystem
	@Override
	public HealthState checkHealth() {
		REVLibError intakeError = intakeConveyor.getRollerError();
		REVLibError shooterError = shooterConveyor.getRollerError();
		if ((intakeError != null && intakeError != REVLibError.kOk)
				|| (shooterError != null && shooterError != REVLibError.kOk)) {
			return HealthState.RED;
		}
		return HealthState.GREEN;
	}
	@Override
	public void mustangPeriodic() {
		intakeConveyor.updateConveyorState();
		shooterConveyor.updateConveyorState();
		checkState();
	}
}

class Conveyor {

	private SparkMAXLite roller;
	private double CONVEYOR_SPEED = 0.8;
	private int ballCount = 0;
	BeamBreak beamBreak;
	public Conveyor(int motorID, int beamBreakID) {
		roller = SparkMAXFactory.buildSparkMAX(motorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
		beamBreak = new BeamBreak(beamBreakID);
	}
	public int getBallCount() {
		return ballCount;
	}
	// Updates the ball count of the conveyor
	public void updateConveyorState() {
		if (beamBreak.isTriggered()) {
			ballCount = 1;

		}
		ballCount = 0;
	}
	// Runs the conveyor in the specified direction
	public void run(boolean intaking) {
		if (!intaking) {
			roller.set(-CONVEYOR_SPEED);
		} else {
			roller.set(CONVEYOR_SPEED);
		}
	}
	// Stops the conveyor
	public void stop() {
		roller.stopMotor();
	}
	// Returns the current roller error
	public REVLibError getRollerError() {
		return roller.getLastError();
	}
}

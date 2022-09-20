package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Timer;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/**
 * Connects the intake to the shooter
 * 
 * @author Armaan, Soham, Edward
 */
public class ConveyorSystem extends MustangSubsystemBase {

	public enum Status {
		OFF,
		INTAKING,
		EJECTING,
		SHOOTING
	}

	private Conveyor intakeConveyor, shooterConveyor;
	private Status status = Status.OFF;
	private Timer timer = new Timer();
	private final int CONVEYOR_IDLE_CHECK_PERIOD = 2;
	private Shooter shooter;

	public ConveyorSystem() {
		setName("ConveyorSystem");
		setLogFileHeader("Status");
		intakeConveyor = new Conveyor(RobotMap.INTAKE_CONVEYOR_MOTOR, RobotMap.INTAKE_CONVEYOR_BEAMBREAK, 0.7); // This is done cuz we were seeing some cases where the ball would go past the beam break and touch the shooter wheel and so it would just blurp it out but different speeds solve it
		shooterConveyor = new Conveyor(RobotMap.SHOOTER_CONVEYOR_MOTOR, RobotMap.SHOOTER_CONVEYOR_BEAMBREAK, 0.6);
	}

	public void setShooter(Shooter shooter){
		this.shooter = shooter;
	}

	public void debugBeamBreaks() {
		intakeConveyor.debugBeamBreaks();
		shooterConveyor.debugBeamBreaks();
	}

	public boolean isRunning(){
		return status != Status.OFF;
	}

	// Actions
	// Runs the Conveyor in the given mode
	public void setConveyorMode(Status mode) {
		if(getStatus() != mode){
			timer.stop();
		}
		if (mode == Status.INTAKING) {
			if(getBallCount() != 2){
				intakeConveyor();
			}
		} else if (mode == Status.SHOOTING) {
			shootConveyor();
			timer.start();
		} else if (mode == Status.EJECTING) {
			ejectConveyor();
			timer.start();
		} else if (mode == Status.OFF) {
			stopAll();
		}
	}

	// Helper method of runconveyor
	private void intakeConveyor() {
		if(intakeConveyor.getBallCount() == 0){
			intakeConveyor.run(true);
		}
		if(shooterConveyor.getBallCount() == 0){
			shooterConveyor.run(true);
		}
		status = Status.INTAKING;
	}

	// Helper method of runconveyor
	private void shootConveyor() {
		intakeConveyor.run(true);
		shooterConveyor.run(true);
		status = Status.SHOOTING;
	}

	// Helper method of runconveyor
	private void ejectConveyor() {
		intakeConveyor.run(false);
		if(getBallCount() == 1){
			shooterConveyor.run(false);
		}
		status = Status.EJECTING;
	}

	// Uses current state of conveyor to determine what parts need to be shut down
	private void checkState() {
		switch (status) {
			case INTAKING:
				if (shooterConveyor.getBallCount() == 1) {
					shooterConveyor.stop();
					if (intakeConveyor.getBallCount() == 1) {
						intakeConveyor.stop();
						status = Status.OFF;
					}
				}
				break;
			case EJECTING:
				if (getBallCount() == 0) {
					if (timer.advanceIfElapsed(CONVEYOR_IDLE_CHECK_PERIOD)) {
						stopAll();
					}
				}
				break;
			case SHOOTING:
				if (getBallCount() == 0) {
					if (timer.advanceIfElapsed(CONVEYOR_IDLE_CHECK_PERIOD)) {
						stopAll();
						shooter.idle();
					}
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
		timer.reset();
		timer.stop();
	}

	// Data collection
	// Returns the total number of balls in the conveyor
	public int getBallCount() {
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

	public Status getStatus() {
		return status;
	}

	@Override
	public void debugSubsystem() {
		debugBeamBreaks();
		writeToLogFile(status.toString());
	}
}

class Conveyor {

	private SparkMAXLite roller;
	private double CONVEYOR_SPEED = 0.7;
	private int ballCount = 0;

	private BeamBreak beamBreak;

	public Conveyor(int motorID, int beamBreakID, double conveyorSpeed) {
		roller = SparkMAXFactory.buildSparkMAX(motorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
		beamBreak = new BeamBreak(beamBreakID);
		CONVEYOR_SPEED = conveyorSpeed;
	}

	public int getBallCount() {
		return ballCount;
	}

	// Updates the ball count of the conveyor
	public void updateConveyorState() {
		if (beamBreak.isTriggered()) {
			ballCount = 1;
			return;

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

	public void debugBeamBreaks() {
		beamBreak.sendBeamBreakDataToDashboard();
	}
}

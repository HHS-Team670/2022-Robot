package frc.team670.robot.commands.routines.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Rotates the drivebase to an angle and then starts the conveyor
 * 
 * @author laksh
 */
public class AlignAngleToTargetAndShoot extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private ConveyorSystem conveyor;
    private Shooter shooter;
    private Vision vision;
    private double targetAngle;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double relativeYawToTarget;

    double prevCapTime;

    private final double ANGULAR_P = 0.03;
    private final double ANGULAR_D = 0.0;
    private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private double rotationSpeed;
    private double heading;

    private boolean foundTarget = false;
    private boolean alreadyAlligned = false;

    public AlignAngleToTargetAndShoot(DriveBase driveBase, Vision vision, ConveyorSystem conveyor, Shooter shooter) {
        this.driveBase = driveBase;
        this.conveyor = conveyor;
        this.vision = vision;
        this.shooter = shooter;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
        addRequirements(conveyor, shooter);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    /**
     * Gets the relative angle to the target from camera when possible, converts to
     * absolute angle, and sets that to the turret's target position.
     * 
     * If vision data is unavailable, use position on the field to determine the
     * approximate angle to the target and account for heading to turn.
     * 
     */
    @Override
    public void initialize() {
        if (vision.hasTarget()) {
            relativeYawToTarget = vision.getLastValidAngleCaptured();
            heading = driveBase.getHeading();
            targetAngle = heading - relativeYawToTarget;
            turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
            turnController.enableContinuousInput(-180, 180);
            foundTarget = true;
        }
        if(relativeYawToTarget < 3){
            alreadyAlligned = true;
        }
        shooter.setRPM();
        shooter.run();
    }

    @Override
    public void execute() {
        heading = driveBase.getHeading();
        rotationSpeed = MathUtil.clamp(turnController.calculate(heading, targetAngle), -0.3, 0.3); // Max speed can be
                                                                                                   // 0.3
        if (rotationSpeed > 0 && rotationSpeed < 0.15) {
            rotationSpeed = 0.15;
        } else if (rotationSpeed < 0 && rotationSpeed > -0.15) {
            rotationSpeed = -0.15;
        }

        if (shooter.isUpToSpeed() && (!foundTarget || (Math.abs(driveBase.getHeading() - targetAngle) <= 3))
                && !conveyor.isRunning()) {
            conveyor.runConveyor(ConveyorSystem.Status.SHOOTING);
            Logger.consoleLog("Balls shot Shooter speed: %s", shooter.getVelocity());
        }

        // Logger.consoleLog("Rotation speed: %s target angle: %s, heading %s",
        // rotationSpeed, targetAngle, heading);
        driveBase.curvatureDrive(0, heading < targetAngle ? -rotationSpeed : -rotationSpeed, true); // 0.3 is just a
                                                                                                    // constant safe
        // quick-turn rotational speed
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed() && (alreadyAlligned || !foundTarget || (Math.abs(driveBase.getHeading() - targetAngle) <= 3));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop();
        conveyor.runConveyor(ConveyorSystem.Status.SHOOTING);
        // driveBase.initDefaultCommand();
    }

}
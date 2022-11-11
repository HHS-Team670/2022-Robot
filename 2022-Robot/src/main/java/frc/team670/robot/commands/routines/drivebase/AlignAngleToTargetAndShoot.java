package frc.team670.robot.commands.routines.drivebase;

import java.util.HashMap;
import java.util.Map;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.drivebase.AlignAngleToTarget;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Rotates the drivebase to an angle and then sets conveyor to Shooting mode
 * 
 * @author lakshbhambhani, Justin Hwang, Ethan Chang
 */
public class AlignAngleToTargetAndShoot extends AlignAngleToTarget {

    private ConveyorSystem conveyor;
    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double relativeYawToTarget;

    double prevCapTime;

    private boolean alreadyAligned = false;

    // For debugging purposes; Uncomment the Logger message in end()
    private double startTimeMillis;
    private double initialYaw;
    private double distToTarget;


    double shooterRPM;

    public AlignAngleToTargetAndShoot(DriveBase driveBase, Vision vision, ConveyorSystem conveyor, Shooter shooter) {
        super(driveBase, vision);
        
        this.conveyor = conveyor;
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
        super.initialize();
        
        if (vision.hasTarget()) {
            distToTarget = vision.getDistanceToTargetM();
            initialYaw = relativeYawToTarget;
        }

        if (Math.abs(relativeYawToTarget) < 1)
            alreadyAligned = true;
        else
            alreadyAligned = false;

        startTimeMillis = System.currentTimeMillis();
        shooterRPM = shooter.setRPM();
        shooter.run();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed()
                && (alreadyAligned || super.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.getCamera().takeInputSnapshot();
        vision.getCamera().takeOutputSnapshot();
        // conveyor.setConveyorMode(conveyor);

        //Logger.consoleLog("Time for Align + Shoot: %s Initial Angle: %s Final Angle: %s Interrupted: %s, Distance: %s ShooterSpeed: %s", (System.currentTimeMillis() - startTimeMillis), initialYaw, relativeYawToTarget, interrupted, distToTarget, shooterRPM);
    }

}
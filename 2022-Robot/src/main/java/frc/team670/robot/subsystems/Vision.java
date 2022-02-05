package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ctychen, lakshbhambhani
 */
public class Vision extends MustangSubsystemBase{

    private Solenoid cameraLEDs = new Solenoid(RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);

    private PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    public Pose2d pose, transformedPose, targetPose;

    private double distance;
    private double angle;
    private double visionCapTime;
    private boolean hasTarget;

    // private boolean isBallTracking;

    public Vision(){
        pose = new Pose2d();
        transformedPose = pose;
        targetPose = new Pose2d(FieldConstants.DISTANCE_TO_GOAL_FROM_START, 0.0, new Rotation2d(0.0));
    }

    public Vision(String cameraName){
        camera = new PhotonCamera(cameraName);
        pose = new Pose2d();
        transformedPose = pose;
        targetPose = new Pose2d(FieldConstants.DISTANCE_TO_GOAL_FROM_START, 0.0, new Rotation2d(0.0));
    }

    // These are for sending vision health to dashboard
    // private static NetworkTableInstance instance = NetworkTableInstance.getDefault();

    public boolean hasTarget(){
        return hasTarget;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    private void processImage() {
        try{
            var result = camera.getLatestResult();

            if(result.hasTargets()){
                hasTarget = true;
                distance = PhotonUtils.calculateDistanceToTargetMeters(
                        RobotConstants.CAMERA_HEIGHT_METERS,
                        FieldConstants.HEIGHT_OF_VISION_TARGET,
                        Units.degreesToRadians(RobotConstants.CAMERA_ANGLE_DEGREES),
                        Units.degreesToRadians(result.getBestTarget().getPitch()));
                angle = camera.getLatestResult().getTargets().get(0).getYaw();
                visionCapTime = Timer.getFPGATimestamp() - result.getLatencyMillis()/1000;
                
                //
                updatePose(angle, RobotConstants.cameraOffset);
                //
            } else {
                hasTarget = false;
            }
            
        } catch(Exception e){
            // MustangNotifications.reportWarning(e.toString());
            Logger.consoleLog("NT for vision not found %s", e.getStackTrace());
        }
    }

    public double getDistanceToTargetM() {
        return hasTarget ? distance : RobotConstants.VISION_ERROR_CODE;
    }

    public double getDistanceToTargetCm() {
        return getDistanceToTargetM() * 100;
    }

    public double getDistanceToTargetInches() {
        return getDistanceToTargetM() * 100 / 2.54;
    }

    public double getAngleToTarget() {
        return hasTarget ? angle : RobotConstants.VISION_ERROR_CODE;
    }

    public VisionMeasurement getVisionMeasurements(double heading, Pose2d cameraOffset) {
        if (hasTarget){
            Pose2d targetOffset = cameraOffset.transformBy(getCamToTargetTrans(heading).inverse());
            return new VisionMeasurement(targetOffset, visionCapTime);
        }
        return null;
    }

    public Transform2d getCamToTargetTrans(double heading) {
        // TOOD: make sure this is correct math (check with Mr.Dias)
        Translation2d camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(angle));
        Transform2d camToTargetTrans = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, targetPose, Rotation2d.fromDegrees(heading));
        return camToTargetTrans;
    }

    public void updatePose(double heading, Pose2d cameraOffset) {
        // Get general pose on the field
        Pose2d newPose = poseMath(FieldConstants.DISTANCE_TO_GOAL_FROM_START, distance, angle);
        this.pose = pose.transformBy(
            new Transform2d(new Translation2d(newPose.getX(), newPose.getY()), 
            new Rotation2d(newPose.getX(),newPose.getY())));
        this.transformedPose = pose.transformBy(new Transform2d(new Pose2d(), getVisionMeasurements(heading, cameraOffset).pose));
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d poseMath(double d_0, double d_f, double theta) {
        // d_0 is the distance from the start position of the robot to the high hub
        // d_f is the distance from the current position of the robot to the high hub
        // theta is the angle from the current position of the robot to the high hub
        // d_c is the distance from the start position of the robot to the current position (calculated by the Law of Cosines)
        // angleStartToCurr is the angle from the start position of the robot to the current position (calculated by the Law of Sines)
        // x is the x-direction transformation of the current position from the starting position
        // y is the y-direction transformation of the current position from the starting position

        double d_c = Math.sqrt((d_0 * d_0) + (d_f * d_f) - (2 * d_0 * d_f * Math.cos(theta)));
        double angleStartToCurr = Math.asin((d_f * Math.sin(theta)) / d_c);
        double x = d_c * Math.cos(angleStartToCurr);
        double y = d_c * Math.sin(angleStartToCurr);

        distance = d_c;
        return new Pose2d(x, y, new Rotation2d(angleStartToCurr));
    }

    public double getVisionCaptureTime() {
        return visionCapTime;
    }

    public void LEDSwitch(boolean on) {
        cameraLEDs.set(on);
    }

    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        //TODO: change to be more useful for debugging vision
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // boolean isBall = SmartDashboard.getBoolean("Is Ball", false);
        // processImage(isBall);
        if (hasTarget) {
            SmartDashboard.putNumber("Distance", distance);
            SmartDashboard.putNumber("Angle", angle);
        }
    }

    public class VisionMeasurement{
        public Pose2d pose;
        public double capTime;

        public VisionMeasurement(Pose2d pose, double capTime){
            this.capTime = capTime;
            this.pose = pose;
        }
    }

}
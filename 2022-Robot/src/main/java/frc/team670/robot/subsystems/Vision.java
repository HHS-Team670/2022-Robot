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

    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    public Pose2d pose, transformedPose, targetPose;

    double distance;
    double angle;
    double visionCapTime;
    boolean hasTarget;

    boolean isBall;

    public Vision(String cam)
    {
        pose = new Pose2d();
        transformedPose = pose;
        targetPose = new Pose2d(FieldConstants.DISTANCE_TO_GOAL_FROM_START, 0.0, new Rotation2d(0.0));
    }

    // These are for sending vision health to dashboard
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();

    public boolean hasTarget(){
        return hasTarget;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    private void processImage(Boolean isBall) {
        if (isBall == null)
        {
            isBall = false;
        }

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
                updatePose(angle, RobotConstants.cameraOffset);
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

    public double getDistanceToTargetInches(){
        return getDistanceToTargetM() * 100 / 2.54;
    }

    public double getAngleToTarget(){
        return hasTarget ? angle : RobotConstants.VISION_ERROR_CODE;
    }

    public VisionMeasurement getVisionMeasurements(double heading, Pose2d cameraOffset) {
        if (hasTarget){
            Pose2d targetOffset = cameraOffset.transformBy(getCamToTargetTrans(heading).inverse());
            return new VisionMeasurement(targetOffset, visionCapTime);
        }
        return null;
    }

    public Transform2d getCamToTargetTrans(double heading)
    {
        Translation2d camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(this.distance, Rotation2d.fromDegrees(this.angle));
        Transform2d camToTargetTrans = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, targetPose, Rotation2d.fromDegrees(heading));
        return camToTargetTrans;
    }

    public void updatePose(double heading, Pose2d cameraOffset)
    {
        // Get general pose on the field
        double[] poseStuff = poseMath(FieldConstants.DISTANCE_TO_GOAL_FROM_START, distance, angle);
        this.pose = pose.transformBy(new Transform2d(new Translation2d(poseStuff[0], poseStuff[1]), new Rotation2d(poseStuff[0], poseStuff[1])));
        this.transformedPose = pose.transformBy(new Transform2d(new Pose2d(), getVisionMeasurements(heading, cameraOffset).pose));
    }

    public Pose2d getPose()
    {
        return pose;
    }

    public double[] poseMath(double d_0, double d_f, double theta)
    {
        // d_0 is the distance from the start position of the robot to the high hub
        // d_f is the distance from the current position of the robot to the high hub
        // theta is the angle from the current position of the robot to the high hub
        // d_c is the distance from the start position of the robot to the current position (calculated by the Law of Cosines)
        // gamma is the angle from the start position of the robot to the current position (calculated by the Law of Sines)
        // x is the x-direction transformation of the current position from the starting position
        // y is the y-direction transformation of the current position from the starting position

        double d_c = Math.sqrt((d_0 * d_0) + (d_f * d_f) - (2 * d_0 * d_f * Math.cos(theta)));
        double angleStartToCurr = Math.asin((d_f * Math.sin(theta)) / d_c);
        double x = d_c * Math.cos(angleStartToCurr);
        double y = d_c * Math.sin(angleStartToCurr);
        return new double[]{x, y, d_c};
    }

    public double getVisionCaptureTime() {
        return visionCapTime;
      }

    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    public void turnOffLEDs() {
        cameraLEDs.set(false);
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
        boolean isBall = SmartDashboard.getBoolean("Is Ball", false);
        processImage(isBall);

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
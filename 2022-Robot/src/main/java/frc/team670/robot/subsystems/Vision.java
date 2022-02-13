package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Solenoid cameraLEDs = new Solenoid(RobotMap.PCMODULE, PneumaticsModuleType.CTREPCM, RobotMap.VISION_LED_PCM);
    private PhotonCamera camera = new PhotonCamera(RobotConstants.VISION_CAMERA);
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    private double distance;
    private double angle;
    private double visionCapTime;
    private boolean hasTarget;

    /*
    * 11' 8" --> trying to do 35 feet --tested with 30 feet
    * 2020 camera to target closest distance: 250 in
    * Input:
    * Exposure: 0; Brightness: 24
    * Resolution: 
    * Threshold:
    * H: 27-100; S: 100-255; V: 126-255
    * Contours:
    * Area: 0-100; Ratio: 0-8.7; Fullness: 60-100
    * Speckle Rejection: 100; Target Grouping: 2ormore
    */ 

    //should not have a constructor

    // public Vision(){
    //     // pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    //     // transformedPose = pose;
    //     // targetPose = new Pose2d(FieldConstants.DISTANCE_TO_GOAL_FROM_START, 0.0, new Rotation2d(0.0));
    // }

    // public Vision(String cameraType){
    //     camera = new PhotonCamera(cameraType);
    //     // pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    //     transformedPose = pose;
    //     targetPose = new Pose2d(FieldConstants.DISTANCE_TO_GOAL_FROM_START, 0.0, new Rotation2d(0.0));
    // }

    public void setCameraName (String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

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
                        FieldConstants.HEIGHT_HIGH_GOAL,
                        Units.degreesToRadians(RobotConstants.CAMERA_ANGLE_DEGREES),
                        Units.degreesToRadians(result.getBestTarget().getPitch()));
                angle = camera.getLatestResult().getTargets().get(0).getYaw();
                visionCapTime = Timer.getFPGATimestamp() - result.getLatencyMillis()/1000;
            } else {
                hasTarget = false;
                // Logger.consoleLog("NO TARGET DETECTED");
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

    public VisionMeasurement getVisionMeasurements(double heading, Pose2d targetPose, Pose2d cameraOffset) {
        if (hasTarget){
            Translation2d camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(angle));
            Transform2d camToTargetTrans = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, targetPose, Rotation2d.fromDegrees(heading));
            Pose2d targetOffset = cameraOffset.transformBy(camToTargetTrans.inverse());
            Pose2d newPose = targetOffset.transformBy(changeInPose(heading));
            return new VisionMeasurement(newPose, visionCapTime);
        }
        return null;
    }

    public void setStartPose(double x, double y, double angle) {
        //TODO: check that coordinate is correct
        startPose = new Pose2d(x, y, new Rotation2d(angle));
    }

    public Transform2d changeInPose(double heading) {
        // d_0 is the distance from the start position of the robot to the high hub
        // d_c is the distance from the start position of the robot to the current position (calculated by the Law of Cosines)
        // angleStartToCurr is the angle from the start position of the robot to the current position (calculated by the Law of Sines)
        double d_0 = startPose.getX(); //TODO: make sure this is the right coordinate
        double d_c = Math.sqrt((d_0 * d_0) + (distance * distance) - (2 * d_0 * distance * Math.cos(heading)));
        double angleStartToCurr = Math.asin((distance * Math.sin(heading)) / d_c);
        double x = d_c * Math.cos(angleStartToCurr); // change in x position
        double y = d_c * Math.sin(angleStartToCurr); // change in y position
        
        return new Transform2d(new Translation2d(x , y), new Rotation2d(angleStartToCurr));
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
        processImage();
        
        if (hasTarget) {
            SmartDashboard.putNumber("Distance", distance);
            SmartDashboard.putNumber("Angle", angle);
            SmartDashboard.putNumber("Vision New Pose X", pose.getX());
            SmartDashboard.putNumber("Vision New Pose Y", pose.getY());
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

    @Override
    public void debugSubsystem() {}

}
package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ethan chang, katia bravo, riya gupta
 */
public class Vision extends VisionSubsystemBase{

    public Vision(PowerDistribution pd) {
        super(pd);
    }

    private PhotonCamera camera = new PhotonCamera(RobotConstants.VISION_CAMERA);
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    private double distanceNoError;
    protected double distance;
    protected double angle;
    protected double visionCapTime;
    protected boolean hasTarget;

    /*
    * 11' 8" --> trying to do 35 feet --tested with 30 feet
    * 2020 camera to target closest distance: 250 in
    * Input:
    * Exposure: 0; Brightness: 24 (0 at 1:52pm facing the gray wall)
    * Resolution: 
    * Threshold:
    * H: 27-100; S: 100-255 (194-255 at 2:27pm facing the gray wall); V: 126-255
    * Contours:
    * Area: 0-100; Ratio: 0-8.7; Fullness: 60-100
    * Speckle Rejection: 100; Target Grouping: 2ormore
    */ 

    

   

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


    public void mustangPeriodic() {
        super.processImage(RobotConstants.CAMERA_HEIGHT_METERS, FieldConstants.HIGH_HUB_HEIGHT, RobotConstants.CAMERA_ANGLE_DEGREES);
        
        if (hasTarget) {
            SmartDashboard.putNumber("Vision Distance", distance);
            SmartDashboard.putNumber("Vision Distance Without Error", distanceNoError);
            SmartDashboard.putNumber("Vision Angle (yaw)", angle);
        }
    }

    private double calculateError(double yaw) {
        // data tables: https://docs.google.com/spreadsheets/d/1dKOo8z_jt7KpYxrLy-mMBJh2OgplZgzeqEY14kFfD_Y/edit?usp=sharing
        // modeled using desmos regression

        double a, b, c, d, f;
        a = -1.599 * Math.pow(10, -7);
        b = -0.00000851186604512;
        c = 0.00110468032682;
        d = -0.00402113407271;
        f = 0.0137700071255;

        // 4th degree polynomial
        return a * Math.pow(yaw, 4) + b * Math.pow(yaw, 3) + c * Math.pow(yaw, 2) + d * yaw + f;

    }


    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
    }

}
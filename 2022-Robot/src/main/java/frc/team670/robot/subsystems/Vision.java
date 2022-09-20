package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;


/**
 * Calculates the distance away from the target using the camera and green light
 * 
 * @author Katia Bravo, Riya Gupta, Ethan Chang
 */
public class Vision extends VisionSubsystemBase {

    /*
    * 11' 8" --> trying to do 35 feet --tested with 30 feet
    * 2020 camera to target closest distance: 250 in
    * Input:
    * Exposure: 0; Brightness: 24 (0 at 1:52pm facing the gray wall)
    * Resolution: 160 x 120 FPS, YUYV 30 FPS
    * Threshold:
    * H: 27-100; S: 100-255 (194-255 at 2:27pm facing the gray wall); V: 126-255
    * Contours:
    * Area: 0-100; Ratio: 0-8.7; Fullness: 60-100
    * Speckle Rejection: 100; Target Grouping: 2ormore
    */ 

    public Vision(PowerDistribution pd) {
        super(pd);
        setName("Vision");
        setLogFileHeader("Distance to target", "leds", "Vision Distance", "Vision Angle (yaw)", "Image Capture Time", "Current Time Stamp");
        setCamera(RobotConstants.VISION_CAMERA_NAME);
        super.getCamera().setDriverMode(true);
        
    }

    public void mustangPeriodic() {
        super.processImage(
            RobotConstants.CAMERA_HEIGHT_METERS, 
            FieldConstants.HIGH_HUB_HEIGHT, 
            RobotConstants.CAMERA_ANGLE_DEGREES);
        
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("distance to target", getDistanceToTargetM());
        SmartDashboard.putBoolean("leds", LEDsTurnedOn());

        if (super.hasTarget()) {
            SmartDashboard.putNumber("Vision Distance", distance);
            SmartDashboard.putNumber("Vision Angle (yaw)", angle);
            SmartDashboard.putNumber("Image Capture Time", getVisionCaptureTime());
            SmartDashboard.putNumber("Current Time stamp", Timer.getFPGATimestamp());
            
            writeToLogFile(getDistanceToTargetM(), LEDsTurnedOn(), distance, angle, getVisionCaptureTime(), Timer.getFPGATimestamp());
        } else {
            writeToLogFile(getDistanceToTargetM(), LEDsTurnedOn(), "No target!", "No target!", "No target!", "No target!");
        }

        

    }

}
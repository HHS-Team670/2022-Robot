package frc.team670.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Units;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ctychen, lakshbhambhani
 */
public class Vision extends MustangSubsystemBase{

    // private Solenoid cameraLEDs = new Solenoid(RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);

    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    // These are for sending vision health to dashboard
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();


    public double getAngleToTarget(){
        try{
            return camera.getLatestResult().getTargets().get(0).getYaw();

        }
        catch(Exception e){
            Logger.consoleLog(e.getMessage());
        }
        return RobotConstants.VISION_ERROR_CODE;
    }

    public double getDistanceToTargetInches(){
        return getDistanceToTargetM() * 100 / 2.54;
    }

    public boolean hasTarget(){
        return camera.getLatestResult().hasTargets();
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public double getDistanceToTargetM() {
        // return getDistanceToTargetCm() / 2.54;
        try{
            var result = camera.getLatestResult();

            if(hasTarget()){
                // double range =
                // PhotonUtils.calculateDistanceToTargetMeters(
                //         RobotConstants.CAMERA_HEIGHT,
                //         FieldConstants.VISION_TARGET_CENTER_HEIGHT,
                //         Units.degreesToRadians(RobotConstants.TILT_ANGLE),
                //         Units.degreesToRadians(result.getBestTarget().getPitch()));
        
                //     return range;
            }
            else{
                return RobotConstants.VISION_ERROR_CODE;
            }

            
        }
        catch(Exception e){
            // MustangNotifications.reportWarning(e.toString());
            Logger.consoleLog("NT for vision not found %s", e.getStackTrace());
        }
       return -1;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetCm() {
        return getDistanceToTargetM() * 100;
    }

    // public void turnOnLEDs() {
    //     cameraLEDs.set(true);
    // }

    // public void turnOffLEDs() {
    //     cameraLEDs.set(false);
    // }

    // public void testLEDS() {
    //     cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    // }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        SmartDashboard.putNumber("Distance", getDistanceToTargetM());
        SmartDashboard.putNumber("Angle", getAngleToTarget());
    }

}
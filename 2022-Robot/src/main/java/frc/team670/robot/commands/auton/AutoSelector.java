package frc.team670.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.AutonPathWithDelay;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;


/**
 * Selects an autonomous routine to run based on choice from driver
 */
public class AutoSelector {
    
    int selectedRoutine = -1;

    
    public AutoSelector() {
    }

    // public static enum AutoRoutine {
    //   ATarmacEdge2Ball(0),

    //   BTarmacEdgeCenter2Ball(1),
    //   BTarmacEdgeLower2Ball(2),
    //   BTarmacHighHubTerminal(3),
    //   UNKNOWN(-1);

    //     private final int ID;

    //     AutoRoutine(int id) {
    //         ID = id;
    //     }

    //     public int getID() {
    //         return this.ID;
    //     }

    //     public static AutoRoutine getById(int id) {
    //         for (AutoRoutine e : values()) {
    //             if (e.getID() == id)
    //                 return e;
    //         }
    //         return UNKNOWN;
    //     }

    // }

    /**
     * Gets the value of the enum for auto routines based on an int input from the
     * driver dashboard.
     * 
     * @return
     */
    public int getSelection() {

      
      Number autoID = NetworkTableInstance.getDefault().getTable("/SmartDashboard").getEntry("auton-chooser").getNumber(-1);
        // Logger.consoleLog("auto path number: %s", autoID);

        this.selectedRoutine = (int)(autoID.intValue());
        // Logger.consoleLog("auto path routine: %s", this.selectedRoutine);
        return this.selectedRoutine;
    }

    public double getDelayTime() {
      // Logger.consoleLog("Inside AutoSelector delay time:" + SmartDashboard.getNumber("delayTime", -1));
      // Logger.consoleLog(String.join(", ", SmartDashboard.getKeys()));
      return NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("delayTime").getDouble(-1);
      
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
    public MustangCommand getCommandFromRoutine(int routine, double delayTime, DriveBase driveBase, Intake intake, ConveyorSystem conveyor, 
    Shooter shooter, Deployer deployer){
        // Logger.consoleLog("Inside getCommandFromRoutine() Auton %s", routine);
        // Logger.consoleLog("Inside getCommandFromRoutine - delay time: " + delayTime);
          switch(routine) {
            case 0:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.ATarmacEdge2Ball, HubType.UPPER)); 
            case 1:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.BTarmacEdgeCenter2Ball, HubType.UPPER));
            case 2:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.BTarmacEdgeLower2Ball, HubType.UPPER));
            case 3:
              return new AutonPathWithDelay(delayTime, new FourBallPath(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.BTarmacHighHubTerminal));

            case 4:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.ATarmacEdge2Ball, HubType.LOWER)); 
            case 5:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.BTarmacEdgeCenter2Ball, HubType.LOWER));
            case 6:
              return new AutonPathWithDelay(delayTime, new Edge2Ball(driveBase, intake, conveyor, shooter, deployer, AutonTrajectory.BTarmacEdgeLower2Ball, HubType.LOWER));
            default:
              return null;
          }
    }

}
package frc.team670.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.AutonPathWithDelay;


/**
 * Selects an autonomous routine to run based on choice from driver
 */
public class AutoSelector {
    
    AutoRoutine selectedRoutine = AutoRoutine.UNKNOWN;

    
    public AutoSelector() {
    }

    public static enum AutoRoutine {
      ATarmacEdge2Ball(0),

      BTarmacEdgeCenter2Ball(1),
      BTarmacEdgeLower2Ball(2),
      BTarmacHighHubTerminal(3),
      UNKNOWN(-1);

        private final int ID;

        AutoRoutine(int id) {
            ID = id;
        }

        public int getID() {
            return this.ID;
        }

        public static AutoRoutine getById(int id) {
            for (AutoRoutine e : values()) {
                if (e.getID() == id)
                    return e;
            }
            return UNKNOWN;
        }

    }

    /**
     * Gets the value of the enum for auto routines based on an int input from the
     * driver dashboard.
     * 
     * @return
     */
    public AutoRoutine getSelection() {

      
      Number autoID = NetworkTableInstance.getDefault().getTable("/SmartDashboard").getEntry("auton-chooser").getNumber(-1);
        Logger.consoleLog("auto path number: %s", autoID);

        this.selectedRoutine = AutoRoutine.getById((int)(autoID.intValue()));
        Logger.consoleLog("auto path routine: %s", this.selectedRoutine);
        return this.selectedRoutine;
    }

    public double getDelayTime() {
      Logger.consoleLog("Inside AutoSelector delay time:" + SmartDashboard.getNumber("delayTime", -1));
      Logger.consoleLog(String.join(", ", SmartDashboard.getKeys()));
      return NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("delayTime").getDouble(-1);
      
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
    public MustangCommand getCommandFromRoutine(AutoRoutine routine,
    double delayTime){
        Logger.consoleLog("Inside getCommandFromRoutine() Auton %s", routine);
        Logger.consoleLog("Inside getCommandFromRoutine - delay time: " + delayTime);
          switch(routine) {
            case ATarmacEdge2Ball:
              return new AutonPathWithDelay(delayTime, "message1"); 
            case BTarmacEdgeCenter2Ball:
              return new AutonPathWithDelay(delayTime, "message2");
            case BTarmacEdgeLower2Ball:
              return new AutonPathWithDelay(delayTime, "message3");
            case BTarmacHighHubTerminal:
              return new AutonPathWithDelay(delayTime, "message3");
            default:
              return new AutonPathWithDelay(delayTime, "DEFAULT");
          }
    }

}
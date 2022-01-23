package frc.team670.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Timer;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.auton.PutMessage;
import frc.team670.robot.constants.FieldConstants;

/**
 * Selects an autonomous routine to run based on choice from driver
 */
public class AutoSelector {

    // private static NetworkTableInstance instance;
    // private static NetworkTable table;

    // private DriveBase driveBase;
    // private Intake intake;
    // private Conveyor conveyor;
    // private Indexer indexer;
    // private Shooter shooter;
    // private Turret turret;
    // private Vision coprocessor;

    // private Timer timer;
    
    AutoRoutine selectedRoutine = AutoRoutine.UNKNOWN;

    /**
     * Initializes this command from the given parameters
     * @param driveBase the drivebase of the robot
     * @param intake the intake of the robot
     * @param conveyor the conveyor of the robot
     * @param shooter the shooter of the robot
     * @param indexer the indexer of the robot
     * @param turret the turret of the robot
     * @param coprocessor the raspberry pi
     */
    // public AutoSelector(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Turret turret, Vision coprocessor){
    public AutoSelector() {
        // instance = NetworkTableInstance.getDefault();
        // table = instance.getTable("SmartDashboard");
        
        // this.driveBase = driveBase;
        // this.intake = intake;
        // this.conveyor = conveyor;
        // this.indexer = indexer;
        // this.shooter = shooter;
        // this.turret = turret;
        // this.coprocessor = coprocessor;

        // timer = new Timer();
    }

    public static enum AutoRoutine {
      LogMessage1(0),

      LogMessage2(1),
      LogMessage3(2),
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

    public static enum StartPosition{
        LEFT, 
        CENTER, 
        RIGHT;
    }

    /**
     * Gets the value of the enum for auto routines based on an int input from the
     * driver dashboard.
     * 
     * @return
     */
     // used to be called select()
    public AutoRoutine getSelection() {

      // RETURNED FALSE
      Logger.consoleLog("SmartDashboard contents: ", SmartDashboard.getKeys(0));
      Logger.consoleLog("contains auton-chooser key %s", SmartDashboard.containsKey("auton-chooser"));
      
        Number autoID = SmartDashboard.getNumber("auton-chooser", -1);
        // timer.start();
      
        // NetworkTableEntry value = table.getEntry("auton-chooser");
        // if (value.getType() != NetworkTableType.kDouble) {
        //   Logger.consoleLog("value: %s" , value.getType());
        //   return this.selectedRoutine;
        // }
        // Number autoID = value.getNumber(-1);
        Logger.consoleLog("auton selector id: %s", autoID);
        this.selectedRoutine = AutoRoutine.getById((int)(autoID.intValue()));
        Logger.consoleLog("auton selector routine: %s", this.selectedRoutine);
        return this.selectedRoutine;
    }

    public double getDelayTime() {
      return SmartDashboard.getNumber("delayTime", -1);
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
     // used to be called getSelectedRoutine()
    public MustangCommand getCommandFromRoutine(AutoRoutine routine,
    double delayTime){
        // AutoRoutine result = select();
        Logger.consoleLog("Auton %s", routine);

          switch(routine) {
            case LogMessage1:
              return new PutMessage(delayTime, "message1"); 
            case LogMessage2:
              return new PutMessage(delayTime, "message2");
            case LogMessage3:
              return new PutMessage(delayTime, "message3");
            
            default:
              return new PutMessage(delayTime, "DEFAULT");
          }
    }

}
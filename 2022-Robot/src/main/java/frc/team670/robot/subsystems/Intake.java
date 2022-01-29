package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {


  private static final double INTAKE_DEPLOYER_SPEED=1.0;
  private static final double INTAKE_ROLLER_SPEED = 1.0; 
  private static final double DEPLOYER_TICKS_NOT_DEPLOYED = 0; // TODO: change this later when we get the motor and subsystem
  private static final double DEPLOYER_TICKS_DEPLOYED = 0;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;

  private double POW = 0.25;

  // SmartMotion constants
  private static final double MAX_ACC = 0;
  private static final double MIN_VEL = 0;
  private static final double MAX_VEL = 0;

  private static final double ALLOWED_ERR = 0;

  private static final double NORMAL_OUTPUT = 0; // Todo: this should be the current output when running normally
  private static final double ROTATIONS_PER_CM = 0; // gearing is 50:1
  private static final double HALF_CM = 0.5 * ROTATIONS_PER_CM;

  private int SMARTMOTION_SLOT = 0;
  
  private double INTAKE_PEAK_CURRENT = 35; // Testing
  private int exceededCurrentLimitCount = 0;
  private SparkMAXLite roller;
  private SparkMAXLite deployer;

  private CANEncoder deployerEncoder;
  private CANPIDController deployerController;
  
  private boolean isDeployed = false; //TODO: true for testing, change this

  public Intake() {
    // Intake roller should be inverted
    roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
    deployer = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_DEPLOYER, Motor_Type.NEO_550);
    roller.setInverted(true);
    roller.setOpenLoopRampRate(1.0);
    deployerEncoder = deployer.getEncoder();
    deployerEncoder.setPosition(DEPLOYER_TICKS_NOT_DEPLOYED);
    deployerController = deployer.getPIDController();

    deployerController.setP(kP);
    deployerController.setI(kI);
    deployerController.setD(kD);
    deployerController.setFF(kFF);
    deployerController.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
    deployerController.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
    deployerController.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
    deployerController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
  }

  public boolean isRolling() { 
    return roller.get() != 0;
  }

  public void deploy(boolean isDeployed) {
    this.isDeployed = isDeployed;
    deployer.set(INTAKE_DEPLOYER_SPEED);
    deployerController.setReference(DEPLOYER_TICKS_DEPLOYED, ControlType.kSmartMotion);
  }

  public boolean isDeployed() {
    return isDeployed;
  }

  public void roll(boolean reversed) {
     if (isDeployed) {
      if (reversed) {
        roller.set(INTAKE_ROLLER_SPEED * -1);
      } else {
        roller.set(INTAKE_ROLLER_SPEED);
      }
     }
  }

  public boolean isJammed(){
    double intakeCurrent = roller.getOutputCurrent();
    if (intakeCurrent > 0.2){
      if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
        exceededCurrentLimitCount++;
      } else {
        exceededCurrentLimitCount = 0;
      }
      if (exceededCurrentLimitCount >= 4){ // 4 consecutive readings higher than peak
        return true;
      }
    }

    return false;

  }

  public void stop() {
    roller.stopMotor();
    retractIntake();  
  }

  public void retractIntake () {
    deployer.set(INTAKE_DEPLOYER_SPEED * -1);
    deployerController.setReference(DEPLOYER_TICKS_NOT_DEPLOYED, ControlType.kSmartMotion);
  }

  /**
   * @return RED if the roller has issues, or the intake isn't deployed but the
   *     pneumatics have issues
   */
  @Override
  public HealthState checkHealth() {
    if (roller == null || isSparkMaxErrored(roller)) {
      return HealthState.RED;
    }
    if (deployer == null) {
      if (isDeployed) {

        return HealthState.YELLOW;
      }

      return HealthState.RED;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    checkHealth();
  }

}
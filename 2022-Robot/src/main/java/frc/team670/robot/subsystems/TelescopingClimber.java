package frc.team670.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * 
 * @author Pallavi, ctychen, Sanatan
 */
public class TelescopingClimber {

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

  private SparkMaxPIDController leadController;
  private SparkMaxPIDController leadEncoder;
  private ArrayList<SparkMAXLite> motors;

  private boolean onBar;
  private Double target;

  private int currentAtHookedCount;

  public float motorRotationsAtRetracted;
  public float motorRotationsAtMaxExtension;

  private float softLimitAtRetracted;
  private float softLimitAtExtension;

  public double MAX_EXTENDING_HEIGHT_CM; // TODO: change this later

  public TelescopingClimber(int motorId, double p, double i, double d, double ff, float motorRotationsAtRetracted,
      float motorRotationsAtMaxExtension, double maxExtendingHeightCm) {
    kP = p;
    kI = i;
    kD = d;
    kFF = ff;
    this.motorRotationsAtRetracted = motorRotationsAtRetracted;
    this.motorRotationsAtMaxExtension = motorRotationsAtMaxExtension;
    this.softLimitAtRetracted = this.motorRotationsAtRetracted + .5f;
    this.softLimitAtExtension = motorRotationsAtMaxExtension - 10;

    MAX_EXTENDING_HEIGHT_CM = maxExtendingHeightCm;

    motors = new ArrayList<SparkMAXLite>();
    motors.add(SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO));
    for (SparkMAXLite motor : motors) {
      motor.setIdleMode(IdleMode.kBrake);
      motor.setInverted(true);
      motor.enableSoftLimit(SoftLimitDirection.kForward, true);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      motor.setSoftLimit(SoftLimitDirection.kForward, softLimitAtExtension);
      motor.setSoftLimit(SoftLimitDirection.kReverse, softLimitAtRetracted);
    }
    leadController = motors.get(0).getPIDController();
    leadEncoder = (SparkMaxPIDController) motors.get(0).getEncoder();
    ((RelativeEncoder) leadEncoder).setPosition(this.motorRotationsAtRetracted);

    setDefaultPID();

    onBar = false;
    target = 0.0;
    currentAtHookedCount = 0;
    SmartDashboard.putNumber("Climber power", 0.0);
    SmartDashboard.putBoolean("Climber deploy", false);
    SmartDashboard.putNumber("Climber motor rotations", 0);
    SmartDashboard.putNumber("Climber rotation target", 0);
  }

  public void setDefaultPID() {
    leadController.setP(kP);
    leadController.setI(kI);
    leadController.setD(kD);
    leadController.setFF(kFF);
    leadController.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
    leadController.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
    leadController.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
    leadController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
  }

  public void hookOnBar() {
    if (isHooked() && !onBar) {
      onBar = true;
    }

    if (!onBar) {
      setPower(-1 * POW);
    }
  }

  public void unhookFromBar() {
    if (onBar) {
      setPower(POW);
      onBar = false;
    }
  }

  private boolean isHooked() {
    double current = motors.get(0).getOutputCurrent();
    if (current > 0.2) {
      if (current >= NORMAL_OUTPUT) {
        currentAtHookedCount++;
      } else {
        currentAtHookedCount = 0;
      }
      if (currentAtHookedCount >= 4) { // 4 consecutive readings higher than peak
        return true;
      }
    }
    return false;

  }

  public boolean isHookedOnBar() {
    return onBar;
  }

  public void setPower(double power) {
    motors.get(0).set(power);
  }

  public void climb(double heightCM) {
    double rotations = heightCM * ROTATIONS_PER_CM;
    SmartDashboard.putNumber("Climber rotation target", rotations);
    target = rotations;
    leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
  }

  public HealthState checkHealth() {
    for (SparkMAXLite motor : motors) {
      if ((motor == null || motor.getLastError() != REVLibError.kOk)) {
        return HealthState.RED;
      }
    }
    return HealthState.GREEN;
  }

  public boolean isAtTarget() {
    return (Math.abs(((RelativeEncoder) leadEncoder).getPosition() - target) < HALF_CM);
  }

  protected double getUnadjustedMotorRotations() {
    return ((RelativeEncoder) this.leadEncoder).getPosition();
  }

  protected double getMotorCurrent(int motor) {
    return this.motors.get(motor).getOutputCurrent();
  }

  public void test() {
    setPower(SmartDashboard.getNumber("Climber power", 0.0));
    SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
  }

}
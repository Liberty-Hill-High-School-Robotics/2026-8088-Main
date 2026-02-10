package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

// all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class Turret extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  /* ex:
  private CANSparkMax barRotatorSparkMax;
  private SparkLimitSwitch barReverseLimitSwitch;
  public static RelativeEncoder barRotatorRelativeEncoder;
  PIDController barPID = new PIDController(BarConstants.bP, BarConstants.bI, BarConstants.bD);
  */

  private SparkFlex turretPivot;
  private SparkLimitSwitch turretForwardLimit;
  private SparkLimitSwitch turretReverseLimitSwitch;
  private RelativeEncoder turretEncoder;
  SparkClosedLoopController turretController;

  public Turret() {
    // config motor settings here

    /*
    ex:
    barRotatorSparkMax = new CANSparkMax(CanIDs.barRotatorID, MotorType.kBrushless);
    //barRotatorSparkMax.restoreFactoryDefaults();
    barRotatorSparkMax.setInverted(true);
    barRotatorSparkMax.setIdleMode(IdleMode.kBrake);
    barRotatorSparkMax.setSmartCurrentLimit(40);

    barReverseLimitSwitch = barRotatorSparkMax.getReverseLimitSwitch(Type.kNormallyOpen);

    barRotatorSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    barRotatorSparkMax.setSoftLimit(SoftLimitDirection.kForward, BarConstants.fLimit);

    barRotatorRelativeEncoder = barRotatorSparkMax.getEncoder();
    */ 

    turretPivot = new SparkFlex(CanIDs.kTurretPivot, MotorType.kBrushless);
    turretForwardLimit = turretPivot.getForwardLimitSwitch();
    turretReverseLimitSwitch = turretPivot.getReverseLimitSwitch();
    turretEncoder = turretPivot.getEncoder();
    turretController = turretPivot.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();

    // Set PID gains
    config
    .closedLoop
        .p(MotorSpeeds.kTurretP)
        .i(MotorSpeeds.kTurretI)
        .d(MotorSpeeds.kTurretD)
        .feedForward
            .kS(MotorSpeeds.kTurretS)
            .kV(MotorSpeeds.kTurretV)
            .kA(MotorSpeeds.kTurretA);

    // Set MAXMotion parameters
    config.closedLoop.maxMotion
    .cruiseVelocity(MotorSpeeds.kTurretCruise)
    .maxAcceleration(MotorSpeeds.kTurretAccel)
    .allowedProfileError(MotorSpeeds.kTurretError);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  // as well as check for limits and reset encoders,
  // return true/false if limit is true, or encoder >= x value

  public void goToSetpoint(double setPoint) {
    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
    turretController.setSetpoint(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void turretStop() {
    turretPivot.set(0);
  }

  public void turretRight() {
    turretPivot.set(MotorSpeeds.kTurretSpeed);
  }

  public void turretLeft() {
    turretPivot.set(-MotorSpeeds.kTurretSpeed);
  }
}

package frc.robot.subsystems;

// all imports here
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  SparkClosedLoopController turretController;

  public Turret() {
    // config motor settings here
    turretPivot = new SparkFlex(CanIDs.kTurretPivot, MotorType.kBrushless);
    turretForwardLimit = turretPivot.getForwardLimitSwitch();
    turretReverseLimitSwitch = turretPivot.getReverseLimitSwitch();
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
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .cruiseVelocity(MotorSpeeds.kTurretCruise)
        .maxAcceleration(MotorSpeeds.kTurretAccel)
        .allowedProfileError(MotorSpeeds.kTurretError);

    turretPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches

    SmartDashboard.putBoolean("Forward Limit", turretForwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit", turretReverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Turret Velocity", turretPivot.getEncoder().getVelocity());
    SmartDashboard.putNumber("Turret Position", turretPivot.getEncoder().getPosition());

    if (turretReverseLimitSwitch.isPressed()) {
      turretPivot.getEncoder().setPosition(0);
    }

    if (turretForwardLimit.isPressed()) {
      turretPivot.getEncoder().setPosition(10); // TODO: get a real number from physical turret
    }
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

  public void goToSetpoint(Pose2d setPoint) {
    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control

    // get position of the robot
    double driveX = SmartDashboard.getNumber("Drive X", 0);
    double driveY = SmartDashboard.getNumber("Drive Y", 0);
    double driveOm = SmartDashboard.getNumber("Drive Om", 0);



    double turretX = driveX + Constants.kTurretXOffset;
    double turretY = driveY + Constants.kTurretYOffset;
    turretPivot.getEncoder().getPosition()/10 // 10:1 reduction TODO: test with actual turret
    double turretOm = driveOm + ; 

    // translation from field cords to turret rotation
    double x = setPoint.getX();
    double y = setPoint.getY();

    turretController.setSetpoint(0.0, SparkBase.ControlType.kMAXMotionPositionControl);
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

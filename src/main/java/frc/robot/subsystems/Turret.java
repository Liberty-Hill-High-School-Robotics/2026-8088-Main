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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class Turret extends SubsystemBase {
  // motors & variables here, define them and create any PIDs needed

  private SparkFlex turretPivot;
  private SparkLimitSwitch turretForwardLimit;
  private SparkLimitSwitch turretReverseLimitSwitch;
  SparkClosedLoopController turretController;

  private Field2d turretField = new Field2d();

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
    SmartDashboard.putNumber(
        "Turret Position",
        turretPivot.getEncoder().getPosition()); // TODO: check if in ticks or rotations

    if (turretReverseLimitSwitch.isPressed()) {
      turretPivot.getEncoder().setPosition(0);
    }

    if (turretForwardLimit.isPressed()) {
      turretPivot.getEncoder().setPosition(Constants.kTurretForwardLimit);
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

  // Point the turret at a specific point on the field
  public void goToSetpoint(Pose2d setPoint) {
    // get position of the robot
    double driveX = SmartDashboard.getNumber("Drive X", 0);
    double driveY = SmartDashboard.getNumber("Drive Y", 0);
    double driveOm = SmartDashboard.getNumber("Drive Om", 0);

    double turretX = driveX + Constants.kTurretXOffset;
    double turretY = driveY + Constants.kTurretYOffset;
    double turretOm = getTurretFieldAngleDegrees(driveOm);

    Pose2d turretPose = new Pose2d(turretX, turretY, Rotation2d.fromDegrees(turretOm));

    double x = setPoint.getX();
    double y = setPoint.getY();

    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
    turretController.setSetpoint(0.0, SparkBase.ControlType.kMAXMotionPositionControl);

    turretField.setRobotPose(turretPose);
    SmartDashboard.putData(
        "Turret Pose", turretField); // Use to display turret on field for testing
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

  // Returns the current turret angle in degrees
  public double getTurretAngleDegrees() {
    double turretRotations = turretPivot.getEncoder().getPosition() / 10; // 10:1 reduction
    double angleDeg =
        (turretRotations) * 360.0
            - Constants.kTurretZeroOffset; // convert rotations to degrees and apply offset

    return angleDeg;
  }

  // Returns the turret's absolute heading in field coordinates (degrees).
  public double getTurretFieldAngleDegrees(double driveHeadingDeg) {
    double turretDeg = getTurretAngleDegrees();
    double fieldDeg =
        driveHeadingDeg + (90 - turretDeg); // combine robot heading and turret relative angle
    return fieldDeg;
  }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
// all imports here
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // motors & variables here, define them and create any PIDs needed

  private SparkFlex turretPivot;
  private SparkLimitSwitch turretForwardLimit;
  private SparkLimitSwitch turretReverseLimitSwitch;
  private SparkClosedLoopController turretController;

  private Field2d turretField = new Field2d();
  private Field2d targetField = new Field2d();

  private double driveX;
  private double driveY;
  private double driveOm;

  private double turretX;
  private double turretY;
  private double turretOm;

  private Pose2d hubSetpoint;
  private Pose2d turretPose;

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

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      hubSetpoint = FieldConstants.kBlueHubPose;
    } else {
      hubSetpoint = FieldConstants.kRedHubPose;
    }
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
      turretPivot.getEncoder().setPosition(Constants.kTurretForwardLimit);
    }

    // get position of the robot
    driveX = SmartDashboard.getNumber("Drive X", 0);
    driveY = SmartDashboard.getNumber("Drive Y", 0);
    driveOm = SmartDashboard.getNumber("Drive Om", 0);

    // calculate the position of the turret based on the robot position and angle
    turretX = Math.cos(Units.degreesToRadians(driveOm)) * Constants.kTurretXOffset + driveX;
    turretY = Math.sin(Units.degreesToRadians(driveOm)) * Constants.kTurretXOffset + driveY;
    turretX = -Math.sin(Units.degreesToRadians(driveOm)) * Constants.kTurretYOffset + driveX;
    turretY = -Math.cos(Units.degreesToRadians(driveOm)) * Constants.kTurretYOffset + driveY;
    turretOm = getTurretFieldAngleDegrees(driveOm);

    turretPose = new Pose2d(turretX, turretY, Rotation2d.fromDegrees(turretOm));

    Logger.recordOutput("Turret/Postion", turretPivot.getEncoder().getPosition(), "rotations");
    Logger.recordOutput(
        "Turret/Setpoint", turretController.getMAXMotionSetpointPosition(), "rotations");
    Logger.recordOutput("Turret/Velocity", turretPivot.getEncoder().getPosition(), "rpm");
    Logger.recordOutput(
        "Turret/VelocitySetpoint", turretController.getMAXMotionSetpointVelocity(), "rpm");
    Logger.recordOutput("Turret/ForwardLimit", turretForwardLimit.isPressed());
    Logger.recordOutput("Turret/ReverseLimit", turretReverseLimitSwitch.isPressed());
    Logger.recordOutput("Turret/Pose", turretPose);
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
  public void turretAirMail() {

    Pose2d airMailSetpoint;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (turretY < hubSetpoint.getY()) { // if turret is below hub
        airMailSetpoint = FieldConstants.kBlueLowAirMail;
      } else {
        airMailSetpoint = FieldConstants.kBlueTopAirMail;
      }
    } else {
      if (turretY < hubSetpoint.getY()) { // if turret is below hub
        airMailSetpoint = FieldConstants.kRedLowAirMail;
      } else {
        airMailSetpoint = FieldConstants.kRedTopAirMail;
      }
    }

    double x = airMailSetpoint.getX();
    double y = airMailSetpoint.getY();

    double robotToTargetY = 0;
    double robotToTargetX = 0;
    double angleToTarget = 0;

    if (turretY < hubSetpoint.getY()) { // if turret is below hub
      robotToTargetY = y - turretY;
      robotToTargetX = x - turretX;
      angleToTarget = Units.radiansToDegrees(Math.atan2(robotToTargetY, robotToTargetX));
    } else {
      robotToTargetY = turretY - y;
      robotToTargetX = turretX - x;
      angleToTarget = Units.radiansToDegrees(Math.atan2(robotToTargetY, robotToTargetX)) - 180;
    }

    double robotToTarget = Math.hypot(robotToTargetX, robotToTargetY);

    SmartDashboard.putNumber("Turret Distance to Target", robotToTarget);

    double angleToTargetRotations = getTargetAngleRotations(driveOm, angleToTarget);

    SmartDashboard.putNumber("Turret Angle to Target Rotations", angleToTargetRotations);

    turretField.setRobotPose(turretPose);
    SmartDashboard.putData(
        "Turret Pose", turretField); // Use to display turret on field for testing
    targetField.setRobotPose(airMailSetpoint);
    SmartDashboard.putData("Turret Target", targetField);

    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
    turretController.setSetpoint(
        angleToTargetRotations, SparkBase.ControlType.kMAXMotionPositionControl);

    if (angleToTargetRotations + Constants.kTurretAllowedError
            < turretPivot.getEncoder().getPosition()
        && angleToTargetRotations - Constants.kTurretAllowedError
            > turretPivot.getEncoder().getPosition()) {
      SmartDashboard.putBoolean("Turret On Target", true);
      Logger.recordOutput("Turret/OnTarget", true);
    } else {
      SmartDashboard.putBoolean("Turret On Target", false);
      Logger.recordOutput("Turret/OnTarget", false);
    }
  }

  public void turretPointAtHub() {
    double x = hubSetpoint.getX();
    double y = hubSetpoint.getY();

    double robotToTargetY = 0;
    double robotToTargetX = 0;
    double angleToTarget = 0;

    if (turretY < y) {
      robotToTargetY = y - turretY;
      robotToTargetX = x - turretX;
      angleToTarget = Units.radiansToDegrees(Math.atan2(robotToTargetY, robotToTargetX));
    } else {
      robotToTargetY = turretY - y;
      robotToTargetX = turretX - x;
      angleToTarget = Units.radiansToDegrees(Math.atan2(robotToTargetY, robotToTargetX)) - 180;
    }

    double robotToTarget = Math.hypot(robotToTargetX, robotToTargetY);

    SmartDashboard.putNumber("Turret Distance to Target", robotToTarget);

    double angleToTargetRotations = getTargetAngleRotations(driveOm, angleToTarget);

    SmartDashboard.putNumber("Turret Angle to Target Rotations", angleToTargetRotations);

    turretField.setRobotPose(turretPose);
    SmartDashboard.putData(
        "Turret Pose", turretField); // Use to display turret on field for testing
    targetField.setRobotPose(hubSetpoint);
    SmartDashboard.putData("Turret Target", targetField);

    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
    turretController.setSetpoint(
        angleToTargetRotations, SparkBase.ControlType.kMAXMotionPositionControl);

    if (angleToTargetRotations + Constants.kTurretAllowedError
            < turretPivot.getEncoder().getPosition()
        && angleToTargetRotations - Constants.kTurretAllowedError
            > turretPivot.getEncoder().getPosition()) {
      SmartDashboard.putBoolean("Turret On Target", true);
    } else {
      SmartDashboard.putBoolean("Turret On Target", false);
    }
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

  public void turretToSetpoint(double overideSetpoint) {
    turretController.setSetpoint(overideSetpoint, SparkBase.ControlType.kMAXMotionPositionControl);
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

  // Returns an encoder value based on the target angle
  public double getTargetAngleRotations(double driveOm, double angleToTargetField) {
    double motorRotations =
        (((driveOm + 90.0 - angleToTargetField) + Constants.kTurretZeroOffset) / 360.0) * 10.0;
    return motorRotations;
  }
}

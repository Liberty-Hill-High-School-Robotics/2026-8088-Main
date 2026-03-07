package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  private SparkFlex shooterMotor;
  private SparkClosedLoopController shooterController;
  private SparkFlex shooterBackMotor;
  private SparkClosedLoopController shooterBackController;

  public Shooter() {
    // config motor settings here
    shooterMotor = new SparkFlex(CanIDs.kShooterMotor, MotorType.kBrushless);
    shooterController = shooterMotor.getClosedLoopController();
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kShooterP)
        .i(MotorSpeeds.kShooterI)
        .d(MotorSpeeds.kShooterD)
        .feedForward
        .kS(MotorSpeeds.kShooterS)
        .kV(MotorSpeeds.kShooterV)
        .kA(MotorSpeeds.kShooterA);
    config.inverted(true);
    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterBackMotor = new SparkFlex(CanIDs.kShooterBackMotor, MotorType.kBrushless);
    shooterBackController = shooterBackMotor.getClosedLoopController();
    SparkFlexConfig backConfig = new SparkFlexConfig();
    backConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kShooterBackP)
        .i(MotorSpeeds.kShooterBackI)
        .d(MotorSpeeds.kShooterBackD)
        .feedForward
        .kS(MotorSpeeds.kShooterBackS)
        .kV(MotorSpeeds.kShooterBackV)
        .kA(MotorSpeeds.kShooterBackA);
    backConfig.inverted(false);
    shooterBackMotor.configure(
        backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches
    Logger.recordOutput(
        "Shooter/ShooterFront/ShooterActual", shooterMotor.getEncoder().getVelocity(), "rpm");
    Logger.recordOutput(
        "Shooter/ShooterFront/ShooterSetpoint", shooterController.getSetpoint(), "rpm");
    Logger.recordOutput(
        "Shooter/ShooterBack/ShooterActual", shooterBackMotor.getEncoder().getVelocity(), "rpm");
    Logger.recordOutput(
        "Shooter/ShooterBack/ShooterSetpoint", shooterBackController.getSetpoint(), "rpm");

    SmartDashboard.putNumber("Shooter Actual", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Back Actual", shooterBackMotor.getEncoder().getVelocity());
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

  public void shootAtSpeed() {
    double distance = SmartDashboard.getNumber("Turret Distance to Target", 0);
    double setpoint = 179.836 + (595.497 * distance) + (264.868 * Math.pow(distance, 2));
    double shooterBackingRatio =
        4.5667
            - (2.51262 * distance)
            + (0.332342 * Math.pow(distance, 2)); // TODO: Equation for backing need to be re-done
    setpoint = Math.min(setpoint, 6500);
    double backSetpoint = Math.min(shooterBackingRatio * setpoint, 6500);
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);
    shooterController.setSetpoint(setpoint, ControlType.kVelocity);
    shooterBackController.setSetpoint(backSetpoint, ControlType.kVelocity);
  }

  public void shooterStop() {
    shooterMotor.set(0);
    shooterBackMotor.set(0);
  }
}

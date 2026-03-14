package frc.robot.subsystems;

// all imports here
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  private SparkFlex intakeMotor;

  private SparkFlex indexMotor;
  private SparkFlex indexMotor2;

  public Intake() {
    // config motor settings here
    intakeMotor = new SparkFlex(CanIDs.kIntakeMotor, MotorType.kBrushless);
    indexMotor = new SparkFlex(CanIDs.kIndexMotor, MotorType.kBrushless);
    indexMotor2 = new SparkFlex(CanIDs.kIndexMotor2, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches

    SmartDashboard.putNumber("IndexSpeed", indexMotor.getEncoder().getVelocity());
    Logger.recordOutput("Intake/Index/IndexSpeed", indexMotor.getEncoder().getVelocity(), "rpm");
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

  public void intakeIn() {
    intakeMotor.set(MotorSpeeds.kIntakeSpeed * 1.5);
    indexMotor.set(MotorSpeeds.kIntakeSpeed);
    indexMotor2.set(MotorSpeeds.kIntakeSpeed);
  }

  public void intakeUp() {
    intakeMotor.set(MotorSpeeds.kIntakeSpeed * 1.5);
    indexMotor.set(-MotorSpeeds.kIntakeSpeed);
    indexMotor2.set(-MotorSpeeds.kIntakeSpeed);
  }

  public void intakeOut() {
    intakeMotor.set(-MotorSpeeds.kIntakeSpeed * 1.5);
    indexMotor.set(-MotorSpeeds.kIntakeSpeed);
    indexMotor2.set(-MotorSpeeds.kIntakeSpeed);
  }

  public void intakeStop() {
    intakeMotor.set(0);
    indexMotor.set(0);
    indexMotor2.set(0);
  }
}

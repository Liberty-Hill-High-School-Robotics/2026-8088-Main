package frc.robot.subsystems;

// all imports here
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class Intake extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  /* ex:
  private CANSparkMax barRotatorSparkMax;
  private SparkLimitSwitch barReverseLimitSwitch;
  public static RelativeEncoder barRotatorRelativeEncoder;
  PIDController barPID = new PIDController(BarConstants.bP, BarConstants.bI, BarConstants.bD);
  */
  private SparkFlex intakeMotor1;

  private SparkFlex intakeMotor2;

  public Intake() {
    // config motor settings here
    intakeMotor1 = new SparkFlex(CanIDs.kIntakeMotor1, MotorType.kBrushless);
    intakeMotor2 = new SparkFlex(CanIDs.kIntakeMotor2, MotorType.kBrushless);
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

  public void intakeIn() {
    intakeMotor1.set(MotorSpeeds.kIntakeSpeed);
    intakeMotor2.set(MotorSpeeds.kIntakeSpeed);
  }

  public void intakeUp() {
    intakeMotor1.set(MotorSpeeds.kIntakeSpeed);
    intakeMotor2.set(-MotorSpeeds.kIntakeSpeed);
  }

  public void intakeOut() {
    intakeMotor1.set(-MotorSpeeds.kIntakeSpeed);
    intakeMotor2.set(-MotorSpeeds.kIntakeSpeed);
  }

  public void intakeStop() {
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }
}

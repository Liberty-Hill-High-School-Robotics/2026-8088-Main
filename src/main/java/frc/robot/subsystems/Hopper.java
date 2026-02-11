package frc.robot.subsystems;

// all imports here
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class Hopper extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  private SparkMax hopperMotor;

  public Hopper() {
    // config motor settings here
    hopperMotor = new SparkMax(CanIDs.kHopperMotor, MotorType.kBrushless);
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

  public void hopperIn() {
    hopperMotor.set(MotorSpeeds.kHopperSpeed);
  }

  public void hopperOut() {
    hopperMotor.set(-MotorSpeeds.kHopperSpeed);
  }

  public void hopperStop() {
    hopperMotor.set(0);
  }
}

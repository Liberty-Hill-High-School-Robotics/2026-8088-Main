package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Hopper.HopperShoot;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class BallEject extends ParallelCommandGroup {

  public BallEject(Intake m_intake, Hopper m_hopper) {

    addCommands(new IntakeOut(m_intake), new HopperShoot(m_hopper));
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

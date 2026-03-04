package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Hopper.HopperIn;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class BallToHopper extends ParallelCommandGroup {

  public BallToHopper(Intake m_intake, Hopper m_hopper) {

    addCommands(new IntakeIn(m_intake), new HopperIn(m_hopper));
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

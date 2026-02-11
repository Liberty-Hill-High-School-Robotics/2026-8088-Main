package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class SpinOut extends Command {
  // The subsystem the command runs on
  private final Hopper m_hopper;

  public SpinOut(Hopper subsystem) {
    m_hopper = subsystem;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.spinOut();
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.hopperStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

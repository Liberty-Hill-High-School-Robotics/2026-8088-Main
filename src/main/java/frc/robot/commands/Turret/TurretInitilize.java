package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class TurretInitilize extends Command {
  // The subsystem the command runs on
  private final Turret m_turret;
  private boolean goRight;

  public TurretInitilize(Turret subsystem, boolean goRight) {
    m_turret = subsystem;
    this.goRight = goRight;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (goRight) {
      m_turret.turretRight();
    } else {
      m_turret.turretLeft();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.initLimits();
    m_turret.turretStop();
  }

  @Override
  public boolean isFinished() {
    return m_turret.getTurretReverseLimit() || m_turret.getTurretForwardLimit();
  }
}

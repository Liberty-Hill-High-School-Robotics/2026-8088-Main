package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class TurretToSetpoint extends Command {
  // The subsystem the command runs on
  private final Turret m_turret;
  private double overrideSetpoint;

  public TurretToSetpoint(Turret subsystem, double overrideSetpoint) {
    m_turret = subsystem;
    this.overrideSetpoint = overrideSetpoint;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_turret.turretToSetpoint(overrideSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.turretStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

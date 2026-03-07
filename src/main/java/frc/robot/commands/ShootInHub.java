package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter.ShootAtSpeed;
import frc.robot.commands.Turret.TurretPointAtHub;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ShootInHub extends ParallelCommandGroup {

  public ShootInHub(Intake m_intake, Turret m_turret, Shooter m_shooter, Hopper m_hopper) {

    addCommands(
        new BallToShoot(m_intake, m_hopper),
        new TurretPointAtHub(m_turret),
        new ShootAtSpeed(m_shooter));
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

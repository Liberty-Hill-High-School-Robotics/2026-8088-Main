package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter.ShootAtSpeed;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AirMail extends ParallelCommandGroup {

  public AirMail(Intake m_intake, Shooter m_shooter, Hopper m_hopper) {

    double distance = SmartDashboard.getNumber("Turret Distance to Target Mail", 2);

    if (SmartDashboard.getBoolean(
        "Drive On Target Mail", true)) { // Turret pointing is pointing at the air mail spot
      addCommands(
          new BallToShoot(m_intake, m_hopper), // Pass balls to shoot
          // new TurretAirMail(m_turret), // point the turret the air mail spot
          new ShootAtSpeed(m_shooter, distance)); // keep shooter

    } else { // Turret is not pointed at air mail spot
      addCommands(
          new BallToHopper(m_intake, m_hopper), // Store balls until ready
          // new TurretAirMail(m_turret), // point the turret the air mail spot
          new ShootAtSpeed(m_shooter, distance)); // spin up shooter
    }
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

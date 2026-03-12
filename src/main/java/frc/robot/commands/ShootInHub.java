package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter.ShootAtSpeed;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ActiveHubTracker;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ShootInHub extends ParallelCommandGroup {

  private ActiveHubTracker hubTracker;

  public ShootInHub(Intake m_intake, Shooter m_shooter, Hopper m_hopper, boolean doSmartShoot) {

    hubTracker = new ActiveHubTracker();
    double distance = SmartDashboard.getNumber("Turret Distance to Target Hub", 2);

    if (SmartDashboard.getBoolean(
        "Drive On Target Hub", true)) { // drive pointing is pointing at the hub
      if (doSmartShoot) { // using smart shooting
        if (hubTracker.isHubActive()) { // hub is active
          addCommands(
              new BallToShoot(m_intake, m_hopper), // Pass balls to shoot
              // new TurretPointAtHub(m_turret), // keep turret pointed at hub
              new ShootAtSpeed(m_shooter, distance)); // keep shooting
        } else { // hub is not active
          addCommands(
              new BallToShoot(m_intake, m_hopper), // store balls until active
              // new TurretPointAtHub(m_turret), // keep turret aligned
              new ShootAtSpeed(m_shooter, distance)); // spin up shooter
        }
      } else { // not using smart shooting
        addCommands(
            new BallToShoot(m_intake, m_hopper), // Pass balls to shoot
            // new TurretPointAtHub(m_turret), // keep turret pointed at hub
            new ShootAtSpeed(m_shooter, distance)); // keep shooting
      }
    } else { // Turret is not pointed at hub
      addCommands(
          new BallToHopper(m_intake, m_hopper), // Store balls until ready
          // new TurretPointAtHub(m_turret), // point the turret at hub
          new ShootAtSpeed(m_shooter, distance)); // spin up shooter
    }
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

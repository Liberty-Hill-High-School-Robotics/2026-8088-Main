package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DetectAndIntake extends ParallelCommandGroup {

  public DetectAndIntake(Drive m_drive, Intake m_intake, Hopper m_hopper) {
    PIDController targetPID = new PIDController(.02, 0, 0);
    addCommands(
        new BallToHopper(m_intake, m_hopper),
        DriveCommands.joystickDriveRobot(
            m_drive,
            () -> 0.7,
            () -> 0.0,
            () -> -targetPID.calculate(0, SmartDashboard.getNumber("objYaw", 0))));
    targetPID.close();
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}

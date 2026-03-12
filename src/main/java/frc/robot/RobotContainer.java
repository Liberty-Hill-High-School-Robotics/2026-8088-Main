// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AirMail;
import frc.robot.commands.BallEject;
import frc.robot.commands.BallToHopper;
import frc.robot.commands.DetectAndIntake;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.ShootInHub;
import frc.robot.commands.Shooter.ShootAtSpeed;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Vision m_vision;
  // private final Turret m_turret;
  private final Hopper m_hopper;
  private final Intake m_intake;
  private final Shooter m_shooter;

  // Controller
  // See https://www.chiefdelphi.com/t/command-scheduler-loop-overrun-causes-and-remedies/457999
  private final CommandGenericHID m_driverController =
      new CommandGenericHID(OIConstants.kDriverControllerPort);

  private final CommandGenericHID m_operatorController =
      new CommandGenericHID(OIConstants.kOperatorControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        m_vision = new Vision(m_drive::addVisionMeasurement);
        // m_turret = new Turret();
        m_hopper = new Hopper();
        m_intake = new Intake();
        m_shooter = new Shooter();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_vision = new Vision(m_drive::addVisionMeasurement);
        // m_turret = new Turret();
        m_hopper = new Hopper();
        m_intake = new Intake();
        m_shooter = new Shooter();
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement);
        // m_turret = new Turret();
        m_hopper = new Hopper();
        m_intake = new Intake();
        m_shooter = new Shooter();
        break;
    }

    // Named Comands for PathPlanner autos
    NamedCommands.registerCommand("IntakeIn", new IntakeIn(m_intake));
    NamedCommands.registerCommand(
        "ShootInHub", new ShootInHub(m_intake, m_shooter, m_hopper, false));
    // NamedCommands.registerCommand("InitilizeTurretLeft", new TurretInitilize(m_turret, false));
    // NamedCommands.registerCommand("InitilizeTurretRight", new TurretInitilize(m_turret, true));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> -m_driverController.getRawAxis(4)));

    // add button bindings here
    /*
     * ex:
     * final Trigger CommandName = m_drivercontroller.button();
     * CommandName.toggleontrue(new commandFromCode(m_subsystem(s)));
     */

    // Setup for Driver

    // Switch to X pattern when X button is pressed
    final Trigger XPattern = m_driverController.button(3);
    XPattern.onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when B button is pressed
    final Trigger ResetHeading = m_driverController.button(2);
    ResetHeading.onTrue(
        Commands.runOnce(
                () ->
                    m_drive.setPose(
                        new Pose2d(m_drive.getPose().getTranslation(), Rotation2d.kZero)),
                m_drive)
            .ignoringDisable(true));

    // Lock to 0° when Y button is held
    final Trigger JoystickDriveAtZero = m_driverController.button(4);
    JoystickDriveAtZero.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> Rotation2d.kZero));

    // Lock to 45° when left trigger is held, for use when crossing the trench
    final Trigger DriveCrossTrench = m_driverController.axisGreaterThan(2, .1);
    DriveCrossTrench.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> Rotation2d.fromDegrees(45)));

    // Automatic Intake when Right trigger is held
    final Trigger AutoIntake = m_driverController.axisGreaterThan(3, .1);
    AutoIntake.whileTrue(new DetectAndIntake(m_drive, m_intake, m_hopper));

    // Setup for Operator

    /*
    // Shoots balls into hub when right trigger is pressed, considers hub state, slow down drive
    final Trigger ShootInHub = m_operatorController.axisGreaterThan(3, .1);
    ShootInHub.whileTrue(new ShootInHub(m_intake, m_turret, m_shooter, m_hopper, true));
    ShootInHub.whileTrue(DriveCommands.joystickDrive(
        m_drive,
        () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
        () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
        () -> m_driverController.getRawAxis(4) * Constants.kDriveShootingRatio));

    // Shoots balls into hub when right bumper is pressed, does not consider hub state, slow down drive
    final Trigger ShootInHubDumb = m_operatorController.button(6);
    ShootInHubDumb.whileTrue(new ShootInHub(m_intake, m_turret, m_shooter, m_hopper, false));
    ShootInHubDumb.whileTrue(DriveCommands.joystickDrive(
        m_drive,
        () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
        () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
        () -> m_driverController.getRawAxis(4) * Constants.kDriveShootingRatio));

    // Shoots balls into Aliance Zone when Left trigger is pressed, does not consider hub state, slow down drive
    final Trigger AirMail = m_operatorController.axisGreaterThan(2, .1);
    AirMail.whileTrue(new AirMail(m_intake, m_turret, m_shooter, m_hopper));
    AirMail.whileTrue(DriveCommands.joystickDrive(
        m_drive,
        () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
        () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
        () -> m_driverController.getRawAxis(4) * Constants.kDriveShootingRatio));
    */

    final Trigger ShootInHub = m_operatorController.axisGreaterThan(3, .1);
    ShootInHub.whileTrue(new ShootInHub(m_intake, m_shooter, m_hopper, true));
    ShootInHub.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
            () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
            () ->
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Robot Angle to Target Hub", 0))));

    final Trigger ShootInHubDumb = m_operatorController.button(6);
    ShootInHubDumb.whileTrue(new ShootInHub(m_intake, m_shooter, m_hopper, false));
    ShootInHubDumb.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
            () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
            () ->
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Robot Angle to Target Hub", 0))));

    final Trigger AirMail = m_operatorController.axisGreaterThan(2, .1);
    AirMail.whileTrue(new AirMail(m_intake, m_shooter, m_hopper));
    AirMail.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getRawAxis(1) * Constants.kDriveShootingRatio,
            () -> -m_driverController.getRawAxis(0) * Constants.kDriveShootingRatio,
            () ->
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Robot Angle to Target Mail", 0))));

    // Ejects balls out of the robot while Y button is held
    final Trigger Eject = m_operatorController.button(4);
    Eject.whileTrue(new BallEject(m_intake, m_hopper));

    // Override to run the intake without obj detection when X button is pressed
    final Trigger IntakeInOverride = m_operatorController.button(3);
    IntakeInOverride.toggleOnTrue(new BallToHopper(m_intake, m_hopper));

    /*
    // Override to move turret left while left on the D-pad is held
    final Trigger TurretLeftOverride = m_operatorController.povLeft();
    TurretLeftOverride.whileTrue(new TurretLeft(m_turret));

    // Override to move turret right while right on the D-pad is held
    final Trigger TurretRightOverride = m_operatorController.povRight();
    TurretRightOverride.whileTrue(new TurretRight(m_turret));

    // Override to center the turret when B button is pressed
    final Trigger TurretCenterOverride = m_operatorController.button(2);
    TurretCenterOverride.toggleOnTrue(
        new TurretToSetpoint(m_turret, Constants.kTurretForwardLimit / 2));
        */

    // Override to shoot at a set Speed when A button is pressed
    final Trigger ShootAtFallbackSpeed = m_operatorController.button(1);
    ShootAtFallbackSpeed.toggleOnTrue(
        new ShootAtSpeed(m_shooter, 2)); // find a consistant distance to fall back on

    /*
    final Trigger ReInitilizeTurret = m_operatorController.button(8);
    ReInitilizeTurret.toggleOnTrue(new TurretInitilize(m_turret, false));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

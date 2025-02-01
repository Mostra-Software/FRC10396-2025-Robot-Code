// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.elevator.AutoScore;
import frc.robot.commands.elevator.HomeElevator;
import frc.robot.commands.elevator.SetElevatorPercent;
import frc.robot.commands.outtake.Intake;
import frc.robot.commands.outtake.Shoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOSpark;
import frc.robot.util.TargetingSystem;
import frc.robot.util.TargetingSystem.ReefBranchLevel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final TargetingSystem targetingSystem;
  private final Leds leds = Leds.getInstance();

  // Controller
  private final CommandXboxController driverJoy = new CommandXboxController(0);

  private final CommandPS5Controller operatorJoy = new CommandPS5Controller(1);
  private Trigger autoScoreGetReady = driverJoy.leftTrigger(0.5);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        targetingSystem = new TargetingSystem();
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                targetingSystem);

        elevator = new Elevator(new ElevatorIOSpark(), targetingSystem);
        outtake = new Outtake(new OuttakeIOSpark());
        break;

      case SIM:
        targetingSystem = new TargetingSystem();
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                targetingSystem);

        elevator = new Elevator(new ElevatorIOSim(), targetingSystem);
        outtake = new Outtake(new OuttakeIOSim());
        break;

      default:
        targetingSystem = new TargetingSystem();
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                targetingSystem);

        elevator = new Elevator(new ElevatorIO() {}, targetingSystem);
        outtake = new Outtake(new OuttakeIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoy.getLeftY(),
            () -> -driverJoy.getLeftX(),
            () -> -driverJoy.getRightX()));

    // Lock to 0° when A button is held
    driverJoy
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAutoSnap(
                drive, () -> -driverJoy.getLeftY(), () -> -driverJoy.getLeftX()));

    // Switch to X pattern when X button is pressed
    //
    driverJoy.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    autoScoreGetReady.onTrue(new AutoScore(elevator, driverJoy.rightTrigger(.5)));

    // Reset gyro to 0° when B button is pressed
    driverJoy
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverJoy.y().onTrue(Commands.runOnce(() -> drive.setPoseFacingReef()).ignoringDisable(true));

    // Elevator Openloop Up
    operatorJoy.povUp().whileTrue(new SetElevatorPercent(0.5, elevator));

    // Elevator Openloop Down
    operatorJoy.povDown().whileTrue(new SetElevatorPercent(-0.5, elevator));

    // Elevator ClosedLoop Controls

    // Home
    operatorJoy.L1().whileTrue(new HomeElevator(elevator));

    // L1
    operatorJoy
        .cross()
        .onTrue(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L1)));

    // L2
    operatorJoy
        .square()
        .onTrue(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L2)));

    // L3
    operatorJoy
        .circle()
        .onTrue(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L3)));

    // L4
    operatorJoy
        .triangle()
        .onTrue(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L4)));

    // Outtake ClosedLoop Shoot
    operatorJoy.R2().whileTrue(new Shoot(outtake));

    // Outtake ClosedLoop Intake
    operatorJoy.L2().whileTrue(new Intake(outtake));
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

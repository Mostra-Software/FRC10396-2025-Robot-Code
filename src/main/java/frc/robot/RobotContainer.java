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

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.climb.SetClimbPercent;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.elevator.AutoScore;
import frc.robot.commands.elevator.HomeElevator;
import frc.robot.commands.elevator.SetElevatorPercent;
import frc.robot.commands.outtake.DeAlg;
import frc.robot.commands.outtake.Intake;
import frc.robot.commands.outtake.RunOuttake;
import frc.robot.commands.outtake.Shoot;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.TargetingSystem;
import frc.robot.util.TargetingSystem.ReefBranchLevel;
import frc.robot.util.TargetingSystem.ReefBranchSide;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({DoublePressTracker.class})
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Climb climb;
  private final TargetingSystem targetingSystem;
  private final Leds leds = Leds.getInstance();
  private final Vision vision;

  // Controller
  private final CommandXboxController driverJoy = new CommandXboxController(1);

  private final CommandPS5Controller operatorJoy = new CommandPS5Controller(2);

  private Trigger autoScoreGetReady = driverJoy.rightTrigger(0.5);

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
        outtake = new Outtake(new OuttakeIOSpark(), targetingSystem);
        climb = new Climb(new ClimbIOSpark());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

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
        outtake = new Outtake(new OuttakeIOSim(), targetingSystem);
        climb = new Climb(new ClimbIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
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
        outtake = new Outtake(new OuttakeIO() {}, targetingSystem);
        climb = new Climb(new ClimbIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

    // Named Commands for Auton
    NamedCommands.registerCommand(
        "L4_Shoot", new AutoScore(elevator, outtake, elevator::isAtSetpoint, targetingSystem));

    NamedCommands.registerCommand(
        "auto_align", new AutoAlign(drive, targetingSystem).withTimeout(2));

    // Event Triggers for Auton
    new EventTrigger("run_intake_trigger")
        .whileTrue(new Intake(outtake, driverJoy, targetingSystem).withTimeout(1.5));

    new EventTrigger("run_shooter_trigger").whileTrue(new Shoot(outtake).withTimeout(1));

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
    /*
         Trigger shouldKeepRunning =
        new Trigger(targetingSystem::isAutoAssistedTeleop)
            .and(targetingSystem::shouldKeepIntakeRunning);
    Trigger shouldRunIntake =
        new Trigger(targetingSystem::shouldRunIntake).and(targetingSystem::isAutoAssistedTeleop);

    shouldRunIntake
        .or(shouldKeepRunning)
        .onTrue(new Intake(outtake, driverJoy, targetingSystem))
        .onFalse(getStopIntakeCommand());

     */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.applyDeadband(-driverJoy.getLeftY(), DriveConstants.driverDeadband),
            () -> MathUtil.applyDeadband(-driverJoy.getLeftX(), DriveConstants.driverDeadband),
            () -> MathUtil.applyDeadband(-driverJoy.getRightX(), DriveConstants.driverDeadband)));

    new Trigger(targetingSystem::isAutoAssistedTeleop)
        .whileTrue(
            DriveCommands.joystickDriveAutoSnap(
                drive,
                () -> MathUtil.applyDeadband(-driverJoy.getLeftY(), DriveConstants.driverDeadband),
                () ->
                    MathUtil.applyDeadband(-driverJoy.getLeftX(), DriveConstants.driverDeadband)));

    driverJoy
        .leftBumper()
        .whileTrue(
            new InstantCommand(() -> targetingSystem.setBranchSide(ReefBranchSide.LEFT))
                .andThen(new AutoAlign(drive, targetingSystem)));

    driverJoy
        .rightBumper()
        .whileTrue(
            new InstantCommand(() -> targetingSystem.setBranchSide(ReefBranchSide.RIGHT))
                .andThen(new AutoAlign(drive, targetingSystem)));

    // Switch to X pattern when X button is pressed
    //
    driverJoy.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    autoScoreGetReady
        .onTrue(new AutoScore(elevator, outtake, driverJoy.leftTrigger(.5), targetingSystem))
        .onFalse(new HomeElevator(elevator).andThen(new RunOuttake(true, 0, outtake)));

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

    // driverJoy.y().onTrue(Commands.runOnce(() ->
    // drive.setPoseFacingReef()).ignoringDisable(true));
    driverJoy.y().whileTrue(getDeAlgeCommand()).onFalse(getDeAlgaeOnFalseCommand());

    // Elevator Openloop Up
    operatorJoy.povUp().whileTrue(new SetElevatorPercent(0.5, elevator));

    // Elevator Openloop Down
    operatorJoy.povDown().whileTrue(new SetElevatorPercent(-0.5, elevator));

    // Elevator ClosedLoop Controls

    // Home
    operatorJoy.L1().whileTrue(new HomeElevator(elevator));

    // Auto Assist Toggle for Teleop
    driverJoy
        .x()
        .doublePress()
        .onTrue(new InstantCommand(() -> targetingSystem.toggleAutoAssist()));
    // L1
    operatorJoy
        .cross()
        .onTrue(
            new InstantCommand(() -> targetingSystem.setCoralMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L1))));

    // L2
    operatorJoy
        .square()
        .onTrue(
            new InstantCommand(() -> targetingSystem.setCoralMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L2))));

    // L3
    operatorJoy
        .circle()
        .onTrue(
            new InstantCommand(() -> targetingSystem.setCoralMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L3))));

    // L4
    operatorJoy
        .triangle()
        .onTrue(
            new InstantCommand(() -> targetingSystem.setCoralMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L4))));

    // L2 Coral
    operatorJoy
        .square()
        .doublePress()
        .onTrue(
            Commands.runOnce(() -> targetingSystem.setAlgaeMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L2))));

    // L3 Coral
    operatorJoy
        .circle()
        .doublePress()
        .onTrue(
            Commands.runOnce(() -> targetingSystem.setAlgaeMode())
                .andThen(Commands.runOnce(() -> targetingSystem.setTarget(ReefBranchLevel.L3))));

    // Outtake Shoot
    operatorJoy.R2().whileTrue(new Shoot(outtake)).onFalse(getStopIntakeCommand());

    // Outtake Intake
    operatorJoy
        .L2()
        .whileTrue(new Intake(outtake, driverJoy, targetingSystem))
        .onFalse(getStopIntakeCommand());

    // Openloop Climb
    operatorJoy.povRight().whileTrue(new SetClimbPercent(0.75, climb));
    operatorJoy.povLeft().whileTrue(new SetClimbPercent(-0.75, climb));

    // deAlg disabled until assembly
    // operatorJoy.R1().whileTrue(new DeAlg(outtake));

    // Manuel Feed for Outtake
    operatorJoy.R1().whileTrue(new RunOuttake(true, 0.15, outtake));
  }

  public TargetingSystem getTargetingSystem() {
    return targetingSystem;
  }

  public ParallelCommandGroup getStopIntakeCommand() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> outtake.runPercent(0), outtake),
        new InstantCommand(() -> driverJoy.setRumble(RumbleType.kBothRumble, 0)),
        new InstantCommand(() -> Leds.getInstance().intaking = false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public SequentialCommandGroup getDeAlgeCommand() {
    return new SequentialCommandGroup(
        new DeAlg(outtake, 110), new RunCommand(() -> outtake.runPercent(0.3), outtake));
  }

  public SequentialCommandGroup getDeAlgaeOnFalseCommand() {
    return new SequentialCommandGroup(new DeAlg(outtake, 3));
  }
}

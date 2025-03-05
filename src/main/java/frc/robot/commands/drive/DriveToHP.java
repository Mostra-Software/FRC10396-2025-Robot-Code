// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TargetingSystem;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToHP extends Command {

  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Auto Align/Rot P");
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 20.0;
  private static final double ANGLE_MAX_ACCELERATION = 40.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private Drive drive;
  private Pose2d targetPose = new Pose2d();
  private Pose2d currPose = new Pose2d();
  private Pose2d delta = new Pose2d(99, 99, new Rotation2d(0));
  private TargetingSystem targetingSystem;

  static {
    ANGLE_KP.initDefault(2.3);
  }

  ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP.get(),
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  PIDController xController = new PIDController(0.75, 0, 0);
  PIDController yController = new PIDController(0.75, 0, 0);

  public DriveToHP(Drive drive, TargetingSystem targetingSystem) {
    this.drive = drive;
    this.targetingSystem = targetingSystem;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    currPose = drive.getPose();
    targetPose = targetingSystem.getAutoHPZone();
    if (targetPose == null) end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleController.setP(ANGLE_KP.get());

    currPose = targetingSystem.getRobotPose();

    targetPose = targetingSystem.getAutoHPZone();
    if (targetPose == null) end(true);

    Logger.recordOutput("TargetingSystem/SetpointPoseX", targetPose.getX());
    Logger.recordOutput("TargetingSystem/SetpointPoseRot", targetPose.getRotation());

    // Get linear velocity
    Translation2d linearVelocity =
        new Translation2d(
            xController.calculate(currPose.getX(), targetPose.getX()),
            yController.calculate(currPose.getY(), targetPose.getY()));
    Logger.recordOutput("TargetingSystem/Auto Align Calculated Velocities", linearVelocity);
    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    Logger.recordOutput("TargetingSystem/Auto Align Chassis Speeds", speeds);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(delta.getX()) < 0.03
        && Math.abs(delta.getY()) < 0.03
        && Math.abs(delta.getRotation().getDegrees()) < 4);
  }
}

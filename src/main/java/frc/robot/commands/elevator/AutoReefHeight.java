// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.TargetingSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoReefHeight extends Command {
  private Elevator elevator;
  private double targetHeight = 0;
  private TargetingSystem targetingSystem;

  public AutoReefHeight(Elevator elevator, TargetingSystem targetingSystem) {
    this.targetingSystem = targetingSystem;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    int targetLevel = elevator.getTargetReef();
    System.out.println(targetLevel);
    if (targetingSystem.isCoralMode()) {
      if (targetLevel == 0) {
        targetHeight = ElevatorConstants.L1Height;
      } else if (targetLevel == 1) {
        targetHeight = ElevatorConstants.L2Height;
      } else if (targetLevel == 2) {
        targetHeight = ElevatorConstants.L3Height;
      } else if (targetLevel == 3) {
        targetHeight = ElevatorConstants.L4Height;
      } else {
        targetHeight = ElevatorConstants.L1Height;
      }
    } else {
      if (targetLevel == 1) {
        targetHeight = ElevatorConstants.L2AlgaeHeight;
      } else if (targetLevel == 2) {
        targetHeight = ElevatorConstants.L3AlgaeHeight;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setHeight(targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

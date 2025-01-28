// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPercent extends Command {

  private Elevator elevator;
  private double percent;

  public SetElevatorPercent(double percent, Elevator elevator) {
    this.elevator = elevator;
    this.percent = percent;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    Leds.getInstance().elevator_moving = true;
  }

  @Override
  public void execute() {
    elevator.runPercent(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().elevator_moving = false;
    elevator.runPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

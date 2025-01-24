// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;

public class RunElevatorOpenLoop extends Command {

  private Elevator elevator;
  private Boolean up;
  private double speed;

  public RunElevatorOpenLoop(Boolean up, double speed, Elevator elevator) {
    this.elevator = elevator;
    this.up = up;
    this.speed = speed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    Leds.getInstance().elevator_moving = true;
  }

  @Override
  public void execute() {
    if (up) {
      elevator.runPercent(speed);
    } else {
      elevator.runPercent(-speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().elevator_moving = false;
    elevator.runPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

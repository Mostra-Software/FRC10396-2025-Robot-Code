// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;

public class SetElevatorHeight extends Command {

  private Elevator elevator;
  private double height;

  public SetElevatorHeight(double height, Elevator elevator) {
    this.elevator = elevator;
    this.height = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    Leds.getInstance().elevator_moving = true;
  }

  @Override
  public void execute() {
    elevator.setHeight(height);
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

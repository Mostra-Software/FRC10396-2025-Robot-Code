// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;

public class RunElevatorClosedLoop extends Command {

  private Elevator elevator;
  private int height;

  public RunElevatorClosedLoop(int height, Elevator elevator) {
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
    switch (height) {

        // Home
      case 0:
        elevator.runElevatorMaxMotion(0);

        // L1
      case 1:
        elevator.runElevatorMaxMotion(0);

        // L2
      case 2:
        elevator.runElevatorMaxMotion(0);

        // L3
      case 3:
        elevator.runElevatorMaxMotion(0);

        // L4
      case 4:
        elevator.runElevatorMaxMotion(0);

        break;

      default:
        elevator.runPercent(0);
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

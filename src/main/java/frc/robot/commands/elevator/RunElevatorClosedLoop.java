// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorClosedLoop extends Command {

  private Elevator elevator;
  private double height;

  public RunElevatorClosedLoop(Elevator elevator, double height) {
    this.elevator = elevator;
    this.height = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.runElevatorMaxMotion(height);
    
  }

  @Override
  public void end(boolean interrupted) {

   
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
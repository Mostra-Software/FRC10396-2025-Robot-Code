// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorHoming extends SequentialCommandGroup {
  private Elevator elevator;
  private int homeSlowpoint = 15;

  public ElevatorHoming(Elevator elevator) {
    this.elevator = elevator;

    addCommands(
        new RunCommand(() -> elevator.runElevatorMaxMotion(homeSlowpoint), elevator).until(elevator::homeSequenceSlowPointReached),
        new RunElevatorOpenLoop(false, 0.2, elevator).until(elevator::homeReached),
        new RunCommand(() -> elevator.resetEncoder(), elevator)
        );
  }
}
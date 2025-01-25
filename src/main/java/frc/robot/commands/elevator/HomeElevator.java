// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeElevator extends SequentialCommandGroup {
  private Elevator elevator;

  public HomeElevator(Elevator elevator) {
    this.elevator = elevator;

    addCommands(
        new RunCommand(() -> elevator.setHeight(ElevatorConstants.preHomingPosition), elevator)
            .until(() -> elevator.isAtSetpoint()),
        new RunCommand(() -> elevator.runPercent(-0.2), elevator)
            .until(() -> elevator.getCurrent() > ElevatorConstants.homingCurrent),
        new RunCommand(() -> elevator.resetEncoder(), elevator));
  }
}

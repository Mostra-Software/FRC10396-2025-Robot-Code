// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {

  public AutoScore(Elevator elevator, BooleanSupplier readyToScore) {

    addCommands(
        // new InstantCommand(() -> System.out.println("Auto Score Initiated")),
        new AutoReefHeight(elevator).until(readyToScore),
        // new InstantCommand(() -> System.out.println("Homing")),
        new HomeElevator(elevator));
  }
}

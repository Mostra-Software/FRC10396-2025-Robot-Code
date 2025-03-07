// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.outtake.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeAlgae extends SequentialCommandGroup {
  private Outtake outtake;

  public HomeAlgae(Outtake outtake) {
    this.outtake = outtake;

    addCommands(
        new InstantCommand(() -> outtake.setHome(false), outtake),
        new RunCommand(() -> outtake.setAngle(15), outtake)
            .until(outtake::isAtSetpoint)
            .withTimeout(3),
        new ParallelDeadlineGroup(
            new WaitCommand(0.05), new RunCommand(() -> outtake.setArmPercent(-0.3), outtake)),
        new RunCommand(() -> outtake.setArmPercent(-0.3), outtake)
            .until(() -> outtake.getArmCurrent() > 30)
            .withTimeout(0.5),
        new InstantCommand(() -> outtake.setArmPercent(0), outtake),
        new InstantCommand(() -> outtake.resetEncoder(), outtake),
        new InstantCommand(() -> outtake.setHome(true), outtake));

    addRequirements(outtake);
  }
}

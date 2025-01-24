// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.outtake.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake extends SequentialCommandGroup {

  private Outtake outtake;

  public Intake(Outtake outtake) {
    this.outtake = outtake;

    addCommands(
        new InstantCommand(() -> Leds.getInstance().intaking = true),
        new RunCommand(() -> outtake.outtakeRunClosedLoopVelocity(100), outtake)
            .until(outtake::coralVisibleRear),
        new RunCommand(() -> outtake.outtakeRunClosedLoopVelocity(50), outtake)
            .until(outtake::coralVisibleFront),
        new RunCommand(() -> outtake.runPercent(0), outtake),
        new InstantCommand(() -> Leds.getInstance().intaking = false));
  }
}

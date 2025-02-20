// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.outtake.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake extends SequentialCommandGroup {

  private Outtake outtake;
  private CommandXboxController driver;

  public Intake(Outtake outtake, CommandXboxController driver) {
    this.outtake = outtake;
    this.driver = driver;

    addCommands(
        new InstantCommand(() -> Leds.getInstance().intaking = true),
        new RunCommand(() -> outtake.runPercent(0.5), outtake).until(outtake::hasGP),
        new RunCommand(() -> outtake.runPercent(0.3), outtake).until(() -> !outtake.hasGP()),
        new RunCommand(() -> outtake.runPercent(-0.2), outtake).withTimeout(0.2),
        new InstantCommand(() -> outtake.runPercent(0), outtake),
        new InstantCommand(() -> Leds.getInstance().intaking = false),
        new RunCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0.4)).withTimeout(0.4),
        new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0.0)));
  }
}

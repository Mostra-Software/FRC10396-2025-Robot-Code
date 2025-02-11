// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.leds.Leds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimbPercent extends Command {

  private Climb climb;
  private double percent;

  public SetClimbPercent(double percent, Climb climb) {
    this.climb = climb;
    this.percent = percent;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    Leds.getInstance().climbing = true;
  }

  @Override
  public void execute() {
    climb.runPercent(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().climbing = false;
    climb.runPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

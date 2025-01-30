// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.outtake.Outtake;

public class RunOuttake extends Command {

  private Outtake outtake;
  private double speed;
  private boolean out;

  public RunOuttake(Boolean out, double speed, Outtake outtake) {
    this.out = out;
    this.speed = speed;
    this.outtake = outtake;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    Leds.getInstance().intaking = true;
  }

  @Override
  public void execute() {
    if (out) {
      outtake.runPercent(speed);
    } else {
      outtake.runPercent(-speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().intaking = false;
    outtake.runPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

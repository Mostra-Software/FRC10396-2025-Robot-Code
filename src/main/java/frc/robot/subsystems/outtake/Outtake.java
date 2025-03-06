// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TargetingSystem;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private TargetingSystem targetingSystem;

  public Outtake(OuttakeIO io, TargetingSystem targetingSystem) {
    this.io = io;
    this.targetingSystem = targetingSystem;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);
    targetingSystem.setGP(hasGP());
  }

  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public boolean hasGP() {
    return inputs.hasGP;
  }

  public void setArmPercent(double percent) {
    io.setArmPercent(percent);
  }

  public void setAngle(double angle) {
    io.setAngle((angle));
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public boolean isAtSetpoint() {
    return inputs.isAtSetpoint;
  }

  public double getArmCurrent() {
    return inputs.currentAmpsdeAlg;
  }

  public void setHome(boolean isHome) {
    io.setHome(isHome);
  }

  public Object isAtPreHomingPos() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isAtPreHomingPos'");
  }
}

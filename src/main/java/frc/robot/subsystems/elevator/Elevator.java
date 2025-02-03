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

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TargetingSystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TargetingSystem targetingSystem;

  public Elevator(ElevatorIO io, TargetingSystem targetingSystem) {
    this.io = io;
    this.targetingSystem = targetingSystem;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.targetBranchLevel = targetingSystem.getTargetBranchLevel().ordinal();
    Logger.processInputs("Elevator", inputs);
  }

  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public int getTargetReef() {
    return inputs.targetBranchLevel;
  }

  public void setHeight(double height) {
    io.setHeight((height));
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public boolean isAtSetpoint() {
    return inputs.isAtSetpoint;
  }

  public boolean isHome() {
    return inputs.isHome;
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public void setHome(boolean isHome) {
    io.setHome(isHome);
  }

  public boolean isAtPreHomingPos(){
    return inputs.isAtPreHomingPos;
  }
}

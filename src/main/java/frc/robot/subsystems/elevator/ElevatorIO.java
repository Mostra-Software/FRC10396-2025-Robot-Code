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

import frc.robot.util.TargetingSystem.ReefBranchLevel;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double setpoint = 0.0;
    public boolean isAtSetpoint = false;
    public boolean isHome = false;
    public int targetBranchLevel = ReefBranchLevel.L1.ordinal();
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setHeight(double height) {}

  public default void resetEncoder() {}

  public default void setHome(boolean isHome) {}
}

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

import static frc.robot.subsystems.outtake.OuttakeConstants.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class OuttakeIOSensors implements OuttakeIO {

  private final LaserCan rearSensor = new LaserCan(SensorID);

  public OuttakeIOSensors() {

    try {
      rearSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      rearSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      rearSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // May need to use LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT if having problems!
    // see: https://docs.thethriftybot.com/lasercan/code-examples/lasercan-frc-example-java
    inputs.hasGP = (rearSensor.getMeasurement().distance_mm == SensorTriggerDistance);
  }
}

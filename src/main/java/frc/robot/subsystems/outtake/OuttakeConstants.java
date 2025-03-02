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

public class OuttakeConstants {

  public static final int SensorID = 61;
  public static final int outtakeCanId = 59;
  public static final int deAlgCanId = 58;
  public static final int currentLimit = 80;

  // PID Constants
  public static final double positionP = 0.05;
  public static final double positionI = 0.0;
  public static final double positionD = 0.0;

  public static final int forwardSoftLimit = 0;
  public static final int reverseSoftLimit = 0;

  public static final double SensorTriggerDistance = 70;

  public static final double PIDtolerance = 0.1;
}

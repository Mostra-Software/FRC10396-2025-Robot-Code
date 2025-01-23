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

public class ElevatorConstants {

  public static final int sensorID = 0;
  public static final int masterCanId = 0;
  public static final int slaveCanId = 0;
  public static final double motorReduction = (50./46.)*9.;
  public static final int currentLimit = 40;

  public static final double homeSequenceSlowPoint = 15;

  public static final double forwardSoftLimit = 200;
  public static final double reverseSoftLimit = 0;

  //Position PID Parameters
  public static final double positionP = 0.4;
  public static final double positionI = 0.0;
  public static final double positionD = 0.0;
  public static final int maxVelocity = 1000;
  public static final int maxAcceleration = 1000;

}

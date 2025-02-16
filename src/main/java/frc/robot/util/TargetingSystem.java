// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import frc.robot.FieldConstants.*;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TargetingSystem {

  private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  private ReefBranch targetBranch;
  private ReefBranchLevel targetBranchLevel;
  private Transform2d robotBranchScoringOffset =
      new Transform2d(Inches.of(12).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
  private Boolean hasGP = false;
  private double elevHeight = 0.0;
  private Pose2d robotPose = new Pose2d();

  public double getTargetBranchHeightMeters() {
    switch (targetBranchLevel) {
      case L2 -> {
        return ReefHeight.L2.height;
      }
      case L3 -> {
        return ReefHeight.L3.height;
      }
      case L4 -> {
        return ReefHeight.L4.height;
      }
    }
    return 0;
  }

  public void updateElevHeight(double height) {
    Logger.recordOutput("TargetingSystem/Elev Height", elevHeight);
    this.elevHeight = height;
  }

  public double getElevHeight() {
    return elevHeight;
    
  }

  public void setGP(boolean hasGP) {
    Logger.recordOutput("TargetingSystem/HasGP", hasGP);
    this.hasGP = hasGP;
  }

  
  public boolean hasGP() {
    return hasGP;
  }

  public void updateRobotPose(Pose2d currPose) {
    robotPose = currPose;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public void setTarget(ReefBranchLevel targetBranchLevel) {
    Logger.recordOutput("TargetingSystem/BranchLevel", targetBranchLevel);
    this.targetBranchLevel = targetBranchLevel;
  }

  public ReefBranchLevel getTargetBranchLevel() {
    if (targetBranchLevel != null) return targetBranchLevel;
    else return ReefBranchLevel.L1;
  }

  public void left() {
    if (targetBranch == ReefBranch.H) {
      targetBranch = ReefBranch.I;
    }
  }

  public Pose2d getTargetPose() {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null)
      scoringPose =
          Reef.branchPositions
              .get(targetBranch.ordinal())
              .get(ReefHeight.L2)
              .toPose2d()
              .plus(robotBranchScoringOffset);
    return AllianceFlipUtil.apply(scoringPose);
  }

  public Pose2d getHPZone() {
    Pose2d robotPose = getRobotPose();
    Pose2d nearestHP =
        robotPose.nearest(
            Arrays.asList(
                AllianceFlipUtil.apply(CoralStation.leftCenterFace),
                AllianceFlipUtil.apply(CoralStation.rightCenterFace)));
    Transform2d delta = robotPose.minus(nearestHP);
    Measure distanceX = delta.getMeasureX();
    Measure distanceY = delta.getMeasureY();
    if (distanceX.abs(Meters) < 2 && distanceY.abs(Meters) < 2)
      return nearestHP.plus(new Transform2d(Translation2d.kZero, Rotation2d.fromDegrees(180)));
    else return null;
  }

  public List<Pose2d> getFlippedReefFaces() {
    if (AllianceFlipUtil.shouldFlip()) {
      return Arrays.asList(
          AllianceFlipUtil.apply(Reef.centerFaces[0]),
          AllianceFlipUtil.apply(Reef.centerFaces[1]),
          AllianceFlipUtil.apply(Reef.centerFaces[2]),
          AllianceFlipUtil.apply(Reef.centerFaces[3]),
          AllianceFlipUtil.apply(Reef.centerFaces[4]),
          AllianceFlipUtil.apply(Reef.centerFaces[5]));
    } else return Arrays.asList(Reef.centerFaces);
  }

  public int getNearestReefFace() {
    int face = 0;
    try {
      List<Pose2d> reefFaces = getFlippedReefFaces();
      Pose2d targetFace = getRobotPose().nearest(reefFaces);
      face = reefFaces.indexOf(targetFace);
    } catch (Exception e) {
    }
    return face;
  }

  public Pose2d getNearestBranch(int side) {
    // 0 is right branch
    // 1 is left branch
    int reefFace = getNearestReefFace();
    Pose2d scoringPose = Pose2d.kZero;
    int branch = (reefFace == 1 )? reefFace*2 : reefFace + (reefFace + 1);
    if (targetBranchLevel != null)
        scoringPose =
            Reef.branchPositions
                .get(branch)
                .get(ReefHeight.L2)
                .toPose2d()
                .plus(robotBranchScoringOffset);
      Logger.recordOutput("TargetingSystem/Nearest Branch", scoringPose);
      return AllianceFlipUtil.apply(scoringPose);

  }

  public boolean isInHpZone(Pose2d pose) {
    return (getHPZone() != null);
  }

  public boolean shouldRunIntake() {
    return isInHpZone(getRobotPose()) && !hasGP();
  }

  public enum ReefBranch {
    A,
    B,
    K,
    L,
    I,
    J,
    G,
    H,
    E,
    F,
    C,
    D
  }

  public enum ReefBranchLevel {
    L1,
    L2,
    L3,
    L4
  }
}

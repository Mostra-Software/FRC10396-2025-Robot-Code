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
import edu.wpi.first.units.Measure;
import frc.robot.FieldConstants.*;
import frc.robot.subsystems.leds.Leds;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TargetingSystem {

  private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  private ReefBranch targetBranch;
  private ReefBranchLevel targetBranchLevel = ReefBranchLevel.L2;
  private ReefBranchSide reefBranchSide = ReefBranchSide.RIGHT;
  private Transform2d robotBranchScoringOffset =
      new Transform2d((0.883 / 2.0) + 0.1, Inches.of(0).in(Meters), Rotation2d.fromDegrees(180));

  private Transform2d leftrobotBranchScoringOffset =
      new Transform2d((0.883 / 2.0) + 0.05, -Inches.of(1.0).in(Meters), Rotation2d.fromDegrees(183));

  private Transform2d rightrobotBranchScoringOffset =
      new Transform2d((0.883 / 2.0) + 0.05, Inches.of(1.0).in(Meters), Rotation2d.fromDegrees(187));
  private Transform2d robotHPOffset =
      new Transform2d((0.883 / 2.0) + 0.1, Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));

  private Transform2d robotHPAutoOffsetPLUS = new Transform2d(0, 1.7, Rotation2d.fromDegrees(0));
  private Transform2d robotHPAutoOffsetMINUS = new Transform2d(0, -1.7, Rotation2d.fromDegrees(0));

  private Boolean hasGP = false;
  private double elevHeight = 0.0;
  private Pose2d robotPose = new Pose2d();
  private RobotState robotState = RobotState.AUTO;
  private boolean shouldKeepIntakeRunning = false;
  private GPMode GPmode = GPMode.CORAL;

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

  public void setCoralMode() {
    GPmode = GPMode.CORAL;
    // Logger.recordOutput("TargetingSystem/RobotMode", GPmode);
  }

  public void setAlgaeMode() {
    GPmode = GPMode.ALGAE;
    // Logger.recordOutput("TargetingSystem/RobotMode", GPmode);
  }

  public boolean isCoralMode() {
    // Logger.recordOutput("TargetingSystem/isCoral", (GPmode == GPMode.CORAL));
    return (GPmode == GPMode.CORAL);
  }

  public void setRobotState(RobotState state) {
    // Logger.recordOutput("TargetingSystem/Robot State", state);
    robotState = state;
  }

  public void toggleAutoAssist() {
    if (robotState == RobotState.AUTO_ASSISTED_TELEOP) {
      setRobotState(RobotState.MANUAL_TELEOP);
    } else if (robotState == RobotState.MANUAL_TELEOP) {
      setRobotState(RobotState.AUTO_ASSISTED_TELEOP);
    }
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public boolean isAutoAssistedTeleop() {
    return robotState == RobotState.AUTO_ASSISTED_TELEOP;
  }

  public void updateElevHeight(double height) {
    // Logger.recordOutput("TargetingSystem/Elev Height", elevHeight);
    this.elevHeight = height;
  }

  public double getElevHeight() {
    return elevHeight;
  }

  public void setBranchSide(ReefBranchSide side) {
    Logger.recordOutput("TargetingSystem/Branch Side", side);
    reefBranchSide = side;
    if (side == ReefBranchSide.LEFT) {
      Leds.getInstance().leftReefSelected = true;
      Leds.getInstance().rightReefSelected = false;

    } else {
      Leds.getInstance().leftReefSelected = false;
      Leds.getInstance().rightReefSelected = true;
    }
  }

  public ReefBranchSide getBranchSide() {
    return reefBranchSide;
  }

  public void setGP(boolean hasGP) {
    // Logger.recordOutput("TargetingSystem/HasGP", hasGP);
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
    // Logger.recordOutput("TargetingSystem/BranchLevel", targetBranchLevel);
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
      return nearestHP.plus(robotHPOffset);
    else return null;
  }

  public Pose2d getAutoHPZone() {
    Pose2d robotPose = getRobotPose();
    Pose2d nearestHP =
        robotPose.nearest(
            Arrays.asList(
                AllianceFlipUtil.apply(CoralStation.leftCenterFace),
                AllianceFlipUtil.apply(CoralStation.rightCenterFace)));
    robotPose = nearestHP.plus(robotHPOffset);
    // Logger.recordOutput("TargetingSystem/HP Auto Pose", robotPose);

    return robotPose;
  }

  public Pose2d getMidHP() {

    Pose2d robotPose = getRobotPose();
    if (robotPose.getMeasureY().in(Meters) < 4.) {
      robotPose = robotPose.plus(robotHPAutoOffsetPLUS);
    } else {
      robotPose = robotPose.plus(robotHPAutoOffsetMINUS);
    }
    if (AllianceFlipUtil.shouldFlip()) {
      robotPose = robotPose.plus(new Transform2d(-0.6, 0, Rotation2d.fromDegrees(0)));
    } else robotPose = robotPose.plus(new Transform2d(0.6, 0, Rotation2d.fromDegrees(0)));

    Logger.recordOutput("TargetingSystem/MidAuto Pose", robotPose);
    return robotPose;
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

  public Pose2d getClosestReefFace() {
    int faceIndex = getNearestReefFace();
    if (faceIndex != -1) {
      return AllianceFlipUtil.apply(Reef.centerFaces[faceIndex].plus(robotBranchScoringOffset));
    } else return new Pose2d();
  }

  public Pose2d getNearestBranchSide() {
    // 0 is right branch
    // 1 is left branch
    int reefFace = getNearestReefFace();
    Pose2d scoringPose = Pose2d.kZero;
    int branch =
        (getBranchSide() == ReefBranchSide.LEFT) ? reefFace * 2 : reefFace + (reefFace + 1);
    if (targetBranchLevel != null)
      scoringPose =
          Reef.branchPositions
              .get(branch)
              .get(ReefHeight.L2)
              .toPose2d()
              .plus(
                  (getBranchSide() == ReefBranchSide.LEFT)
                      ? leftrobotBranchScoringOffset
                      : rightrobotBranchScoringOffset);
    scoringPose = AllianceFlipUtil.apply(scoringPose);
    Logger.recordOutput("TargetingSystem/Nearest Branch", scoringPose);
    return scoringPose;
  }

  public boolean isInHpZone(Pose2d pose) {
    return (getHPZone() != null);
  }

  public boolean shouldRunIntake() {
    return isInHpZone(getRobotPose()) && !hasGP();
  }

  public void setShouldKeepIntakeRunning(boolean keepRunning) {
    shouldKeepIntakeRunning = keepRunning;
  }

  public boolean shouldKeepIntakeRunning() {
    // hp zoneda intakelemeye basladiysa ve
    return shouldKeepIntakeRunning;
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

  public enum RobotState {
    AUTO,
    MANUAL_TELEOP,
    AUTO_ASSISTED_TELEOP
  }

  public enum ReefBranchSide {
    RIGHT,
    LEFT
  }

  public enum GPMode {
    CORAL,
    ALGAE
  }

  public enum ReefBranchLevel {
    L1,
    L2,
    L3,
    L4
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public static void main(String... args) {
    for(int i = 1; i <= 16; i++){
      Pose3d pose = aprilTagFieldLayout.getTagPose(i).get();
      System.out.println(i + ": " + pose.getX() + ", " + pose.getY() + ", " + pose.getZ());
    }
    RobotBase.startRobot(Robot::new);
  }
}

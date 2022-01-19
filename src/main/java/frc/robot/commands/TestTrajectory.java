// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TestTrajectory extends TrajectoryFollowBase {

  final Drivetrain m_drivetrain;

  


  public TestTrajectory(Drivetrain drivetrain) {
    super(TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0,0, Rotation2d.fromDegrees(0)),
      new Pose2d(1,0, Rotation2d.fromDegrees(0)),
      new Pose2d(1,-1,Rotation2d.fromDegrees(0)),
      new Pose2d(0,-1,Rotation2d.fromDegrees(0))),
      
      new TrajectoryConfig(2.5, 1.8 ).addConstraints(List.of(new SwerveDriveKinematicsConstraint(drivetrain.getKinematics()
      , 4.5), new CentripetalAccelerationConstraint(1.9)))
      ), drivetrain);
    //TODO Auto-generated constructor stub
    m_drivetrain = drivetrain;
  }

  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_drivetrain.resetOdometry(new Pose2d());
    m_drivetrain.resetGyro();
  }
}

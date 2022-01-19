// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollowBase extends SwerveControllerCommand {

  public TrajectoryFollowBase(Trajectory trajectory,  Drivetrain drivetrain) {
    super(trajectory, drivetrain::getOdometryPose, drivetrain.getKinematics(), 
    new PIDController(1, 0, 0), new PIDController(1, 0, 0),
     new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)), drivetrain::setModuleStates, drivetrain);
  }
 


}

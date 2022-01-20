// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.NonCenteredSpinTrajectory;
import frc.robot.commands.TestTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and butto\n mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Joystick m_stick = new Joystick(0);

  private final Command m_manualDrive = new Command() {
    ChassisSpeeds commandSpeeds = new ChassisSpeeds(0,0,0);
    @Override
    public void execute(){

      commandSpeeds.vxMetersPerSecond = -2*m_stick.getY();
      commandSpeeds.vyMetersPerSecond = 2*m_stick.getX();
      commandSpeeds.omegaRadiansPerSecond = 3*m_stick.getTwist();
      //m_drivetrain.setSpeed(speeds);
      //m_drivetrain.setSpeedGyroRelative(commandSpeeds);
      m_drivetrain.setSpeedAroundPoint(commandSpeeds);
    }

    @Override
    public Set<Subsystem> getRequirements() {
      // TODO Auto-generated method stub
      return Set.of(m_drivetrain);
    }

    @Override
    public boolean isFinished(){
      return false;
    }
  }; 

  private final Command m_testTrajectory = new TestTrajectory(m_drivetrain);
  private final Command m_NonCenteredSpinTrajectory = new NonCenteredSpinTrajectory(m_drivetrain);

  


  private final Trigger setButton = new JoystickButton(m_stick, 2).whileActiveOnce(new InstantCommand(()->{
    m_drivetrain.resetOdometry(new Pose2d());
    m_drivetrain.resetGyro();
  },m_drivetrain));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_manualDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_stick.setXChannel(0);
    m_stick.setYChannel(1);
    m_stick.setTwistChannel(3);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_NonCenteredSpinTrajectory;
  }
}

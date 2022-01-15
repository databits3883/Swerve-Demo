// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.databits3883.databitslib.sparkmax.SparkMaxPIDController;
import com.databits3883.databitslib.swerveControl.SwerveDrive;
import com.databits3883.databitslib.swerveControl.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import java.util.List;

public class Drivetrain extends SubsystemBase {
  SwerveDrive m_drive;

  AHRS m_gyro;

  SparkMaxLimitSwitch[] triggers = new SparkMaxLimitSwitch[4];
  SwerveModule[] modules = new SwerveModule[4];
  CANSparkMax[] rotationMotors = new CANSparkMax[4];


  static final double[] switchLocations = {-4.122, -2.54,-1.03,0.598};

  static SwerveDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    List<Pair<SwerveModule,Translation2d>> modulesWithLoc = List.of(
      Pair.of(newModule(1,2,"front right"),new Translation2d(driveTrackLength/2,driveTrackWidth/2)),
      Pair.of(newModule(3,4,"rear right"),new Translation2d(-driveTrackLength/2,driveTrackWidth/2)),
      Pair.of(newModule(5,6,"rear left"),new Translation2d(-driveTrackLength/2,-driveTrackWidth/2)),
      Pair.of(newModule(7,8,"front left"),new Translation2d(driveTrackLength/2,-driveTrackWidth/2))
      
    );
    m_drive = new SwerveDrive(modulesWithLoc);

    m_gyro = new AHRS(I2C.Port.kMXP);
    addChild("Gyro", m_gyro);
    SendableRegistry.addLW(this, "drivetrain");

    m_odometry = new SwerveDriveOdometry(m_drive.getKinematics(), Rotation2d.fromDegrees(m_gyro.getAngle()));
  }

  SwerveModule newModule(int velChannel, int angleChannel, String name){
    CANSparkMax rot = new CANSparkMax(angleChannel,MotorType.kBrushless);
    rot.setInverted(true);
    rot.getAlternateEncoder(4096).setInverted(true);
    CANSparkMax vel = new CANSparkMax(velChannel,MotorType.kBrushless);
    SparkMaxPIDController rotC = SparkMaxPIDController.withAlternateEncoder(rot, ControlType.kPosition, 4096);
    rotC.setP(0.7);
    rotC.setI(0);
    rotC.setD(0);
    rotC.setSetpoint(0);
    rotC.setSetpoint(rotC.getSignal());
    SparkMaxPIDController velC = SparkMaxPIDController.withDefaultEncoder(vel, ControlType.kVelocity);
    velC.setP(0.22);
    velC.setI(0);
    velC.setD(1.2);
    velC.setFF(0.23);
    SwerveModule module = new SwerveModule(velC,rotC);
    //addChild(name+" rotation", rotC);
    //addChild(name+" velocity", velC);
    module.setAngleConversionFactor(rotateWheelGear);
    module.setVelocityConversionFactor(velocityWheelGear * wheelCircumfrence * (1.0/60.0));
 
    triggers[velChannel/2] = vel.getReverseLimitSwitch(Type.kNormallyOpen);
    modules[velChannel/2] = module;

    return module;
  }

  public void setSpeed(ChassisSpeeds speeds){
    m_drive.setChassisSpeed(speeds);
  }

  public void setSpeedGyroRelative(ChassisSpeeds speeds){
    m_drive.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond, 
      Rotation2d.fromDegrees(m_gyro.getAngle())
    ));
  }

  public void resetGyro(){
    m_gyro.reset();
    m_odometry.resetPosition(getOdometryPose(), new Rotation2d());
  }

  public void resetOdometry(Pose2d startPosition){
    m_odometry.resetPosition(startPosition, Rotation2d.fromDegrees(m_gyro.getAngle()));
  }

  public Pose2d getOdometryPose(){
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), m_drive.measureCurrentState());
    
    // This method will be called once per scheduler run
    if(RobotState.isDisabled()){
      for(int i=0;i<4;i++){
        if(triggers[i].isPressed()){
          //double calibrationPoint = Math.PI/4.0 +(i*Math.PI/2);
          //modules[i].setWheelAngle(new Rotation2d(calibrationPoint));
          modules[i].setWheelAngle(new Rotation2d(switchLocations[i]));
        }
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("X position", ()->m_odometry.getPoseMeters().getX(), (x)->{return;});
    builder.addDoubleProperty("Y position", ()->m_odometry.getPoseMeters().getY(), (y)->{return;});
  }
}

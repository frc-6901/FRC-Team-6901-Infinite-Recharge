/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ShootBallCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ExampleCommand;
import java.util.List;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

   private final Shooter mShooter = new Shooter();
   private final ShootBallCommand mShootBall = new ShootBallCommand(mShooter);

  //private final Drive m_robotDrive = new Drive(); 


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
     XboxController controller = new XboxController(Constants.controllerPort);
     JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);
     aButton.whenHeld(mShootBall);
  }



  public Command getAutonomousCommand(){
  
    return m_autoCommand;
  }
   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

  //   // Create a voltage constraint to ensure we don't accelerate too fast
  //   var autoVoltageConstraint =
  //       new DifferentialDriveVoltageConstraint(
  //           new SimpleMotorFeedforward(DriveConstants.ksVolts,
  //                                      DriveConstants.kvVoltSecondsPerMeter,
  //                                      DriveConstants.kaVoltSecondsSquaredPerMeter),
  //           DriveConstants.kDriveKinematics,
  //           10);

  //   // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
  //                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(DriveConstants.kDriveKinematics)
  //           // Apply the voltage constraint
  //           .addConstraint(autoVoltageConstraint);

  //   // An example trajectory to follow.  All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(
  //           new Translation2d(3, 0)
            
  //       ),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(5, 3, new Rotation2d(90)),
  //       // Pass config
  //       config
  //   );

    // RamseteCommand ramseteCommand = new RamseteCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose,
    //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //     new SimpleMotorFeedforward(DriveConstants.ksVolts,
    //                                DriveConstants.kvVoltSecondsPerMeter,
    //                                DriveConstants.kaVoltSecondsSquaredPerMeter),
    //     DriveConstants.kDriveKinematics,
    //     m_robotDrive::getWheelSpeeds,
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     // RamseteCommand passes volts to the callback
    //     m_robotDrive::tankDriveVolts,
    //     m_robotDrive
    // );

    // Run path following command, then stop at the end.
  //   return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  // }
}

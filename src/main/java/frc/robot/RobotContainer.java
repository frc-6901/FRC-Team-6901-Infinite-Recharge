/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JogShooter;
import frc.robot.commands.MoveUpstring;
import frc.robot.commands.RunAccelerator;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootAtInitiation;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.TuningShootBall;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.VisionController;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter mShooter = new Shooter();
  private final VisionController mLimelight = VisionController.getInstance();
  
  //private final TuningShootBall mShootBall = new TuningShootBall(mShooter);

   private final Climb mClimber = new Climb();
   private final ClimbDownCommand mClimbDown = new ClimbDownCommand(mClimber);
   private final MoveUpstring mUpstringUp = new MoveUpstring(mClimber, true);
   private final MoveUpstring mUpstringDown = new MoveUpstring(mClimber, false);

  private final Feeder mFeeder = new Feeder();
  
  private final RunAccelerator mAcceleratorCommand = new RunAccelerator(mFeeder);
  private final RunFeeder mFeederCommand = new RunFeeder(mFeeder, false);
  private final RunFeeder mUnjamFeeder = new RunFeeder(mFeeder, true);
  
  private final Superstructure mSuperstructure = new Superstructure(mShooter, mFeeder, mLimelight);
  private final ShootBallCommand mShoot = new ShootBallCommand(mSuperstructure);
  private final TuningShootBall mTuneShoot = new TuningShootBall(mSuperstructure);
  private final JogShooter mJog = new JogShooter(mShooter);
  
  
  private final Intake mIntake = new Intake();
  private final RunIntake mIntakeBalls = new RunIntake(mIntake, true);
  private final RunIntake mOutakeBalls = new RunIntake(mIntake, false);

  private final Drive mRobotDrive = new Drive(); 
  private final TurnToTarget mTurn = new TurnToTarget(mRobotDrive, mLimelight);

  private final XboxController controller = new XboxController(ControllerConstants.controllerPort);
  private final XboxController navigator = new XboxController(ControllerConstants.controllerPort2);

  private final ShootAtInitiation mShootAtInitiation = new ShootAtInitiation(mRobotDrive, mSuperstructure);
  private final DriveForward mDriveForward = new DriveForward(mRobotDrive);

  private SendableChooser<Command> mChooser = new SendableChooser<>();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    mRobotDrive.setDefaultCommand(new RunCommand ( () -> mRobotDrive
      .arcadeDrive(navigator.getY(GenericHID.Hand.kLeft) * DriveConstants.kForwardModifier, navigator.getX(GenericHID.Hand.kRight) * DriveConstants.kTurnModifier), mRobotDrive
    ));

    mChooser.setDefaultOption("Drive Forward", mDriveForward);
    mChooser.addOption("Shoot at Initiation Line", mShootAtInitiation);

    Shuffleboard.getTab("Autonomous").add(mChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton bButton = new JoystickButton(controller, XboxController.Button.kB.value);
    bButton.whenHeld(mUnjamFeeder);
    JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);
    aButton.whenHeld(mFeederCommand);
    JoystickButton xButton = new JoystickButton(controller, XboxController.Button.kX.value);
    xButton.whenHeld(mAcceleratorCommand);
    JoystickButton yButton = new JoystickButton(controller, XboxController.Button.kY.value);
    yButton.whenHeld(mTuneShoot);

    JoystickButton backButton = new JoystickButton(controller, XboxController.Button.kStart.value);
    backButton.whenHeld(mJog);

    JoystickButton leftBumper = new JoystickButton(controller, XboxController.Button.kBumperLeft.value);
    JoystickButton rightBumper = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
    leftBumper.whenHeld(mUpstringUp);
    rightBumper.whenHeld(mUpstringDown);
    
    JoystickButton down = new JoystickButton(controller, XboxController.Button.kStart.value);
    down.whenHeld(mClimbDown);

    JoystickButton navLeftBumper = new JoystickButton(navigator, XboxController.Button.kBumperLeft.value);
    JoystickButton navRightBumper = new JoystickButton(navigator, XboxController.Button.kBumperRight.value);

    navLeftBumper.whenHeld(mOutakeBalls);
    navRightBumper.whenHeld(mIntakeBalls);

    JoystickButton navAButton = new JoystickButton(navigator, XboxController.Button.kA.value);
    navAButton.whenHeld(mTurn);
  }



  public Command getAutonomousCommand(){
  
    return mChooser.getSelected();
  }

}

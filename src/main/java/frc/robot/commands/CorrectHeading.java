/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ControllerWrapper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class CorrectHeading extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive mDrive;
  private final VisionController mLimelight;
  private final ControllerWrapper mController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CorrectHeading(Drive drive, VisionController limelight, ControllerWrapper controller) {
    mDrive = drive;
    mLimelight = limelight;
    mController = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(limelight);
    addRequirements(controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mDrive.arcadeDrive(mController.getY(Hand.kRight), mLimelight.getDriveOutput(DriveConstants.kP, DriveConstants.kD));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mDrive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

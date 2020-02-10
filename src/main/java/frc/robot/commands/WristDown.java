package frc.robot.commands;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class WristDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist mWrist;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristDown(Wrist wrist) {
    mWrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mWrist.moveWrist(-.2);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mWrist.moveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

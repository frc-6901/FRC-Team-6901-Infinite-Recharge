package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;


public class RunAccelerator extends CommandBase {
    private Feeder mFeeder;
    public RunAccelerator(Feeder feeder) {
        mFeeder = feeder;
    }

    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mFeeder.runAccelerator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mFeeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
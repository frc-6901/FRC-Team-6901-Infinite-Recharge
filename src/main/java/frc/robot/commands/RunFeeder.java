package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;


public class RunFeeder extends CommandBase {
    private Feeder mFeeder;
    private boolean mReverse;
    public RunFeeder(Feeder feeder, boolean reverse) {
        mFeeder = feeder;
        mReverse = reverse;
    }

    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mReverse) {
      mFeeder.reverseFeeder();
    } else {
      mFeeder.runFeeder();
    }
      
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
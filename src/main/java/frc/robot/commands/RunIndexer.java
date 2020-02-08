package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends CommandBase {
    private Indexer mIndexer;
    public RunIndexer(Indexer indexer) {
        mIndexer = indexer;
    }

    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mIndexer.runIndexer(IndexerConstants.kIndexerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mIndexer.runIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
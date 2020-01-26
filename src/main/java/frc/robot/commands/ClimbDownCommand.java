package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbDownCommand extends CommandBase{
    private Climb mClimber;
    public ClimbDownCommand(Climb climb) {
        mClimber = climb;
        addRequirements(climb);
    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mClimber.runClimb(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.runClimb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


    
    
}
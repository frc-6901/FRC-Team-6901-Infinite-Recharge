package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climb;

public class MoveUpstring extends CommandBase{
    private Climb mClimber;
    private boolean up;
    public MoveUpstring(Climb climb, boolean isUp) {
        mClimber = climb;
        up = isUp;
        addRequirements(climb);
    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double power = ClimberConstants.kUpstringPower;
      if (!up) power *= -1;    
      mClimber.runUpstring(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.runUpstring(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


    
    
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionController;

public class TurnToTarget extends CommandBase {
    private Drive mDrive;
    private VisionController mVisionController;
    private boolean isTuning;
    public TurnToTarget(Drive driveTrain, VisionController controller, boolean isTuning) {
        mDrive = driveTrain;
        mVisionController = controller;
        this.isTuning = isTuning;
        addRequirements(mDrive);
        addRequirements(mVisionController);
    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output;  
    if (isTuning) {
        output = mVisionController.tuneDriveLimelight(); 
      } else {
        output = mVisionController.getDriveOutput(DriveConstants.kP, DriveConstants.kD);
      }
       
      mDrive.tankDriveVolts(output, -output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mDrive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mVisionController.isAligned();
  }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControllerWrapper;

public class VibrateController extends CommandBase {
    private  ControllerWrapper mController;
    
    public VibrateController(ControllerWrapper controller) {
        mController = controller;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mController.rumble();
    }

  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mController.stopRumble();
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
}
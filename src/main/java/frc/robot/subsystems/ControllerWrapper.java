package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Util.XboxJoystickTrigger;
import frc.Util.XboxPOVButton;
import frc.Util.XboxPOVButton.Arrow;
import frc.robot.Constants.ControllerConstants;
import frc.Util.XboxTrigger;
import frc.Util.XboxJoystickTrigger.Position;


public class ControllerWrapper extends SubsystemBase {
    private XboxController mController;
    public ControllerWrapper(int port) {
        mController = new XboxController(port);
    }

    public void rumble() {
        mController.setRumble(RumbleType.kLeftRumble, .5);
        mController.setRumble(RumbleType.kRightRumble, .5);
    }

    public void stopRumble() {
        mController.setRumble(RumbleType.kLeftRumble, 0);
        mController.setRumble(RumbleType.kRightRumble, 0);
    }

    public Trigger getTrigger(Hand hand) {
        return new XboxTrigger(mController, hand);
    }

    public Trigger getJoystickTrigger(Hand hand, Position position) {
        return new XboxJoystickTrigger(mController, hand, position);
    }

    public Trigger getXboxPOV(Arrow pov) {
        return new XboxPOVButton(mController, pov);
    }

    public double getY(Hand hand) {
       return filterDeadband(mController.getY(hand));

    }

    public XboxController getController() {
        return mController;
    }

    public double getX(Hand hand) {
        return filterDeadband(mController.getX(hand));

    }

    private double filterDeadband(double input) {
        if (Math.abs(input) < ControllerConstants.deadBand) {
            return 0;
        }
        return input;

    }


}
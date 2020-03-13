package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
    XboxController mController;
    Hand kHand;
    public XboxTrigger(XboxController controller, Hand hand) {
        mController = controller;
        kHand = hand;

    }
    @Override
    public boolean get() {
        return mController.getTriggerAxis(kHand) > 0;
    }
}
package frc.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
    private XboxController mController;
    private Hand kHand;
    public XboxTrigger(XboxController controller, Hand hand) {
        mController = controller;
        kHand = hand;

    }
    @Override
    public boolean get() {
        return mController.getTriggerAxis(kHand) > 0;
    }
}
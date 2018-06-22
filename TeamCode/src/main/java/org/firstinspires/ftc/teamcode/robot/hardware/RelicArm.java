package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RelicArm
{
    private final DcMotor extender;
    private final Servo rotator, grabber;

    public RelicArm(DcMotor extender, Servo rotator, Servo grabber)
    {
        this.extender = extender;
        this.rotator = rotator;
        this.grabber = grabber;

        setGrabbingState(false);

        this.rotator.setPosition(0); // bring it in
    }

    public void setExtensionPower(double power)
    {
        extender.setPower(power);
    }

    private boolean up = false;

    public void setRotatorPosition(boolean up)
    {
        this.up = up;

        rotator.setPosition(up ? .361 : .832);
    }

    public void toggleRotator()
    {
        setRotatorPosition(!up);
    }

    private boolean grab = false;

    public void setGrabbingState(boolean grab)
    {
        this.grab = grab;

        grabber.setPosition(grab ? .713 : 0);
    }

    public void toggleGrabbingState()
    {
        setGrabbingState(!grab);
    }
}

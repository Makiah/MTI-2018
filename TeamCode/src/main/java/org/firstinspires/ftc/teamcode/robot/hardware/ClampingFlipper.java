package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class ClampingFlipper
{
    private final Servo leftFlipper, rightFlipper, lowerClamp, upperClamp;
    private final double rightFlipperPush = .85, rightFlipperUp = .7, rightFlipperDown = .15;
    private final double leftFlipperPush = .15, leftFlipperUp = .3, leftFlipperDown = .85; // YOU CAN'T SET THEM TO 0 OR 1
    private final double lowerClampRelease = .21, lowerClampClamp = .35;
    private final double upperClampRelease = .39, upperClampClamp = .57;

    public ClampingFlipper(Servo leftFlipper, Servo rightFlipper, Servo lowerClamp, Servo upperClamp)
    {
        this.leftFlipper = leftFlipper;
        this.rightFlipper = rightFlipper;
        this.lowerClamp = lowerClamp;
        this.upperClamp = upperClamp;

        leftFlipper.setPosition(leftFlipperDown);
        rightFlipper.setPosition(rightFlipperDown);
        lowerClamp.setPosition(lowerClampRelease);
        upperClamp.setPosition(upperClampRelease);
    }

    private int flipperState = 2;
    private boolean midStateTransition = false;
    private long transitionEnd = 0, transitionLength = 0;

    private void setTransitionTime(long ms)
    {
        transitionLength = ms;
        transitionEnd = System.currentTimeMillis() + transitionLength;
    }

    public void attemptFlipperStateIncrement()
    {
        if (midStateTransition)
            return;

        flipperState++;
        if (flipperState > 2) flipperState = 0;

        switch (flipperState)
        {
            case 0: // clamp and flip up.
                setTransitionTime(1000); // 500 ms to clamp and flip up.
                break;

            case 1: // unclamp
                setTransitionTime(600);
                break;

            case 2: // clamp, flip down, unclamp
                setTransitionTime(1000);
                break;
        }

        midStateTransition = true;
        updateMidState(0);
    }

    private void updateMidState(double state)
    {
        switch (flipperState)
        {
            case 0:
                if (state < .001)
                {
                    lowerClamp.setPosition(lowerClampClamp);
                    upperClamp.setPosition(upperClampClamp);
                }
                else if (state > .3)
                {
                    leftFlipper.setPosition(leftFlipperUp);
                    rightFlipper.setPosition(rightFlipperUp);
                }
                break;

            case 1:
                lowerClamp.setPosition(lowerClampRelease);
                upperClamp.setPosition(upperClampRelease);

                leftFlipper.setPosition(leftFlipperPush);
                rightFlipper.setPosition(rightFlipperPush);
                break;

            case 2:
                if (state < .001)
                {
                    lowerClamp.setPosition(lowerClampClamp);
                    upperClamp.setPosition(upperClampClamp);
                }

                if (state > .4)
                {
                    leftFlipper.setPosition(leftFlipperDown);
                    rightFlipper.setPosition(rightFlipperDown);
                }

                if (state > .99)
                {
                    lowerClamp.setPosition(lowerClampRelease);
                    upperClamp.setPosition(upperClampRelease);
                }
                break;
        }
    }

    public void updateFlipperState()
    {
        if (!midStateTransition)
            return;

        if (transitionEnd < System.currentTimeMillis())
        {
            updateMidState(1);
            midStateTransition = false;
        }

        double currentState = 1 - ((1.0 * transitionEnd - System.currentTimeMillis()) / (transitionLength));
        updateMidState(currentState);
    }

    public boolean canIntakeGlyphs()
    {
        return !midStateTransition && flipperState == 2;
    }
}

package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

public class JewelKnocker
{
    private final Servo extender, knocker;
    private final ColorSensor colorSensor;
    private final double extenderIn = .46, extenderOut = .68, extenderHelpKnock = .03;
    private final double knockerLeft = 0, knockerMiddle = .335, knockerRight = .597, knockerIn = 1;

    public JewelKnocker(Servo extender, Servo knocker, ColorSensor jewelSensor, EnhancedOpMode.AutoOrTeleop autoOrTeleop)
    {
        this.extender = extender;
        this.knocker = knocker;
        this.colorSensor = jewelSensor;

        this.extender.setPosition(extenderIn);
        this.knocker.setPosition(knockerIn);
    }

    private double lastRed = 0;

    private boolean isJewelRed()
    {
        lastRed = colorSensor.red();
        return lastRed > 30;
    }

    public void knockJewel(boolean knockRedJewel, Flow flow) throws InterruptedException
    {
        final TimeMeasure jewelKnockPrepTime = new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 1000);
        long start = System.currentTimeMillis();
        double completion = 0;
        while (true)
        {
            completion = (System.currentTimeMillis() - start) / jewelKnockPrepTime.durationIn(TimeMeasure.Units.MILLISECONDS);

            if (completion >= 1)
            {
                extender.setPosition(extenderOut);
                knocker.setPosition(knockerMiddle);
                break;
            }

            // Cube root to cause more gradual movement towards target position near endpoints.
            double x = Math.pow(completion, 1.0/3);

            extender.setPosition(extenderIn + (extenderOut - extenderIn) * x);
            knocker.setPosition(knockerIn + (knockerMiddle - knockerIn) * x);

            flow.yield();
        }

        boolean isJewelRed = isJewelRed();
        boolean knockingRight = isJewelRed() && knockRedJewel;
        this.knocker.setPosition(knockingRight ? knockerRight : knockerLeft);
        this.extender.setPosition(extenderOut + (knockingRight ? extenderHelpKnock : -extenderHelpKnock));
        if (knockingRight)
            this.extender.setPosition(extenderHelpKnock);
        LoggingBase.instance.lines("Red is " + lastRed + " so jewel is " + (isJewelRed ? "red" : "blue"));
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));

        this.extender.setPosition(extenderIn);
        this.knocker.setPosition(knockerIn);
    }
}

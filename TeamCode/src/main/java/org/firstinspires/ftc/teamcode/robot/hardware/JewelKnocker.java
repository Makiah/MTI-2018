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
    private final double extenderIn = .494, extenderOut = .68, extenderHelpKnock = .03;
    private final double knockerLeft = 0, knockerMiddle = .295, knockerMiddleClose = .51, knockerRight = .597, knockerIn = 1;

    public JewelKnocker(Servo extender, Servo knocker)
    {
        this.extender = extender;
        this.knocker = knocker;

        this.extender.setPosition(extenderIn);
        this.knocker.setPosition(knockerIn);
    }

    public void knockJewel(boolean knockRight, Flow flow) throws InterruptedException
    {
        knocker.setPosition(knockerMiddleClose);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 200));

        final TimeMeasure jewelKnockPrepTime = new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 1000);
        long start = System.currentTimeMillis();
        double completion;

        while (true)
        {
            completion = (1.0 * (System.currentTimeMillis() - start)) / jewelKnockPrepTime.durationIn(TimeMeasure.Units.MILLISECONDS);

            if (completion >= 1)
            {
                extender.setPosition(extenderOut);
                knocker.setPosition(knockerMiddle);
                break;
            }

            double x = Math.pow(completion, 1.0/3);

            extender.setPosition(extenderIn + (extenderOut - extenderIn) * x);
            knocker.setPosition(knockerMiddleClose + (knockerMiddle - knockerMiddleClose) * x);

            flow.yield();
        }

        this.knocker.setPosition(knockRight ? knockerRight : knockerLeft);
        this.extender.setPosition(extenderOut + (knockRight ? extenderHelpKnock : -extenderHelpKnock));
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 300));

        this.extender.setPosition(extenderIn);
        this.knocker.setPosition(knockerIn);
    }
}

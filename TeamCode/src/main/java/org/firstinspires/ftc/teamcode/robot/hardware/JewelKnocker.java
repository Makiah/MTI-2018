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
    private final double extenderIn = .46, extenderSemiOut = .52, extenderOut = .68, extenderHelpKnock = .71;
    private final double knockerLeft = 0, knockerMiddle = .28, knockerMiddleCheck = .335, knockerRight = .597, knockerIn = 1;

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
        this.knocker.setPosition(knockerRight);
        this.extender.setPosition(extenderSemiOut);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 300));

        this.knocker.setPosition(knockerMiddle);
        this.extender.setPosition(extenderOut);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 1500));

        this.knocker.setPosition(knockerMiddleCheck);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 100));

        boolean isJewelRed = isJewelRed();
        boolean knockingRight = isJewelRed && knockRedJewel;
        this.knocker.setPosition(knockingRight ? knockerRight : knockerLeft);
        if (knockingRight)
            this.extender.setPosition(extenderHelpKnock);
        LoggingBase.instance.lines("Red is " + lastRed + " so jewel is " + (isJewelRed ? "red" : "blue"));
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));

        this.extender.setPosition(extenderIn);
        this.knocker.setPosition(knockerIn);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));
    }
}

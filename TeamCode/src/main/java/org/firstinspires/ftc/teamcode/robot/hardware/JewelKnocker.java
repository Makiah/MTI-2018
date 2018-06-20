package org.firstinspires.ftc.teamcode.robot.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

public class JewelKnocker
{
    private final Servo dropper, knocker;
    private final ColorSensor colorSensor;
    private final double dropperUp = 0, dropperDown = .581;
    private final double knockerLeft = 1, knockerMiddle = .63, knockerRight = .293, knockerAway = 0;
    public JewelKnocker(Servo dropper, Servo knocker, ColorSensor jewelSensor, EnhancedOpMode.AutoOrTeleop autoOrTeleop)
    {
        this.dropper = dropper;
        this.knocker = knocker;
        this.colorSensor = jewelSensor;

        this.dropper.setPosition(dropperUp);
        this.knocker.setPosition(knockerLeft);
    }

    private double lastRed = 0;

    private boolean isJewelRed()
    {
        lastRed = colorSensor.red();
        return lastRed > 45;
    }

    public void knockJewel(boolean knockRedJewel, Flow flow) throws InterruptedException
    {
        this.knocker.setPosition(knockerMiddle);
        this.dropper.setPosition(dropperDown);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 1500));

        boolean isJewelRed = isJewelRed();
        boolean knockingRight = isJewelRed && knockRedJewel;
        if (!knockingRight)
            this.dropper.setPosition(dropperDown + .03);
        this.knocker.setPosition(knockingRight ? knockerRight : knockerLeft);
        LoggingBase.instance.lines("Red is " + lastRed + " so jewel is " + (isJewelRed ? "red" : "blue"));
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));

        this.dropper.setPosition(dropperUp);
        this.knocker.setPosition(knockerAway);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));
    }

    public void getOuttaTheWay()
    {
        knocker.setPosition(knockerAway);
    }
}

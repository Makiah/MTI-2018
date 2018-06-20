package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

public class JewelKnocker
{
    enum KnockPosition {
        LEFT, MIDDLE, RIGHT
    }

    private final Servo dropper, knocker;
    private final double dropperUp = 0, dropperDown = .6;
    private final double knockerLeft = .89, knockerMiddle = .581, knockerRight = .293, knockerAway = 0;
    public JewelKnocker(Servo dropper, Servo knocker, EnhancedOpMode.AutoOrTeleop autoOrTeleop)
    {
        this.dropper = dropper;
        this.knocker = knocker;

        this.dropper.setPosition(dropperUp);
        this.knocker.setPosition(knockerLeft);
    }

    public void knock(KnockPosition kp, Flow flow) throws InterruptedException
    {
        this.knocker.setPosition(knockerMiddle);
        this.dropper.setPosition(dropperDown);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));

        this.knocker.setPosition(kp == KnockPosition.LEFT ? knockerLeft : knockerRight);
        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 500));

        this.dropper.setPosition(dropperUp);
    }

    public void getOuttaTheWay()
    {
        knocker.setPosition(knockerAway);
    }
}

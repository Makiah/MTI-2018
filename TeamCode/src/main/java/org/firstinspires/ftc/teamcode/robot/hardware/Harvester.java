package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.hardware.OpenRevDcMotorImplEx;

@Config
public class Harvester
{
    public static double currentDrawToTriggerRotation = 1600;
    public static double fixLengthMS = 400;

    private final OpenRevDcMotorImplEx leftHarvest, rightHarvest;

    private enum State {NORMAL, FIXING}
    private State state = State.NORMAL;
    private long fixStartTimestamp = 0, fixEndTimestamp = 0;
    private double currentPower = .4;

    public boolean fixingStateDisabled = true;

    private final ProcessConsole console;

    public Harvester (OpenRevDcMotorImplEx leftHarvest, OpenRevDcMotorImplEx rightHarvest)
    {
        this.leftHarvest = leftHarvest;
        this.rightHarvest = rightHarvest;

        console = LoggingBase.instance.newProcessConsole("Harvester");
    }

    public void run(double power)
    {
        currentPower = power;

        if (state != State.FIXING)
        {
            leftHarvest.setPower(-currentPower);
            rightHarvest.setPower(-currentPower);
        }
    }

    public void toggleFixingStateDisabled()
    {
        setFixingStateDisabled(!fixingStateDisabled);
    }

    public void setFixingStateDisabled(boolean disabled)
    {
        this.fixingStateDisabled = disabled;
    }

    public void update()
    {
        if (fixingStateDisabled)
            return;

        double leftDraw = Math.abs(leftHarvest.getCurrentDraw());

        console.write("Left draw is " + leftDraw);

        if (state == State.FIXING && (System.currentTimeMillis() - fixStartTimestamp) > fixLengthMS)
        {
            state = State.NORMAL;
            run(currentPower);
            fixEndTimestamp = System.currentTimeMillis();
        }

        if (currentPower < 0 && state == State.NORMAL && leftDraw > currentDrawToTriggerRotation && (System.currentTimeMillis() - fixEndTimestamp) > 2000)
        {
            state = State.FIXING;
            fixStartTimestamp = System.currentTimeMillis();

            this.leftHarvest.setPower(-.1);
            this.rightHarvest.setPower(-.4);
        }
    }
}

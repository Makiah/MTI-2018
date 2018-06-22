package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hankutanku.hardware.OpenRevDcMotorImplEx;

@Config
public class Harvester
{
    public static double currentDrawToTriggerRotation = 4;
    public static double fixLengthMS = 400;

    private final OpenRevDcMotorImplEx leftHarvest, rightHarvest;

    private enum State {NORMAL, FIXING}
    private State state = State.NORMAL;
    private long startState = 0;
    private double currentPower = .4;

    public Harvester (OpenRevDcMotorImplEx leftHarvest, OpenRevDcMotorImplEx rightHarvest)
    {
        this.leftHarvest = leftHarvest;
        this.rightHarvest = rightHarvest;
    }

    public void run(double power)
    {
        currentPower = power;

        if (state != State.FIXING)
        {
            leftHarvest.setPower(currentPower);
            rightHarvest.setPower(currentPower);
        }
    }

    public void update()
    {
        if (state == State.FIXING && (System.currentTimeMillis() - startState) > fixLengthMS)
        {
            state = State.NORMAL;
            startState = System.currentTimeMillis();

            leftHarvest.setPower(currentPower);
            rightHarvest.setPower(currentPower);
        }

        if (state == State.NORMAL && (Math.abs(leftHarvest.getCurrentDraw()) > currentDrawToTriggerRotation || Math.abs(rightHarvest.getCurrentDraw()) > currentDrawToTriggerRotation))
        {
            state = State.FIXING;
            startState = System.currentTimeMillis();

            this.leftHarvest.setPower(.4);
            this.rightHarvest.setPower(-.4);
        }
    }
}

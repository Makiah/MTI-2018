package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hankutanku.hardware.OpenRevDcMotorImplEx;

public class Harvester
{
    private final OpenRevDcMotorImplEx leftHarvest, rightHarvest;

    public Harvester (OpenRevDcMotorImplEx leftHarvest, OpenRevDcMotorImplEx rightHarvest)
    {
        this.leftHarvest = leftHarvest;
        this.rightHarvest = rightHarvest;
        this.rightHarvest.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double power)
    {
        leftHarvest.setPower(power);
        rightHarvest.setPower(-power);
    }

    public void update()
    {
    }
}

package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Harvester
{
    private final DcMotor leftHarvest, rightHarvest;

    public Harvester (DcMotor leftHarvest, DcMotor rightHarvest)
    {
        this.leftHarvest = leftHarvest;
        this.leftHarvest.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightHarvest = rightHarvest;
    }

    public void run(double power)
    {
        leftHarvest.setPower(power);
        rightHarvest.setPower(-power);
    }
}

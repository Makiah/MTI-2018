package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@Autonomous(name="Test Weird Strafe", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class RunOffBalanceBoard extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor frontLeft = hardwareMap.dcMotor.get("front left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front right");
        DcMotor backLeft = hardwareMap.dcMotor.get("rear left");
        DcMotor backRight = hardwareMap.dcMotor.get("rear right");

        frontLeft.setPower(.5);
        frontRight.setPower(-.5);
        backLeft.setPower(-1);
        backRight.setPower(1);

        while (!isStopRequested())
            idle();
    }
}

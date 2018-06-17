package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@TeleOp(name="Are Motors Correct?", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class AreMotorsCorrect extends LinearOpMode
{
    class MotorNamePair
    {
        public final DcMotor motor;
        public final String name;
        MotorNamePair(DcMotor motor, String name)
        {
            this.motor = motor;
            this.name = name;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor frontRight, frontLeft, backRight, backLeft;
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("rear right");
        backLeft = hardwareMap.dcMotor.get("rear left");

        DcMotor leftHarvest, rightHarvest;
        leftHarvest = hardwareMap.dcMotor.get("left harvester");
        rightHarvest = hardwareMap.dcMotor.get("right harvester");
        rightHarvest.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorNamePair[] pairs = {
                new MotorNamePair(frontRight, "Front Right Drive Motor"),
                new MotorNamePair(frontLeft, "Front Left Drive Motor"),
                new MotorNamePair(backLeft, "Back Left Drive Motor"),
                new MotorNamePair(backRight, "Back Right Drive Motor"),
                new MotorNamePair(leftHarvest, "Left Harvester"),
                new MotorNamePair(rightHarvest, "Right Harvester")
        };

        int currentPair = -1;
        while (!isStopRequested())
        {
            if (gamepad1.a)
            {
                if (currentPair >= 0)
                    pairs[currentPair].motor.setPower(0);
                sleep(500);
                currentPair++;
                if (currentPair == pairs.length)
                    currentPair = 0;
                telemetry.addLine(pairs[currentPair].name);
                telemetry.update();
                pairs[currentPair].motor.setPower(1);
            }

            idle();
        }
    }
}

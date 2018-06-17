package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

@TeleOp(name="Test Tracking Wheels", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestTrackingWheels extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AbsoluteEncoder leftWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                centerWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                rightWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"));

        while (!isStopRequested())
        {
            telemetry.addLine("Left: " + leftWheel.heading());
            telemetry.addLine("Center: " + centerWheel.heading());
            telemetry.addLine("Right: " + rightWheel.heading());
            telemetry.update();

            idle();
        }
    }
}

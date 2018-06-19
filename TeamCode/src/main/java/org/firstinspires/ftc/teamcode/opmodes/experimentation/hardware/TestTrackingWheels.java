package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.IncrementalAbsoluteEncoder;

import java.text.DecimalFormat;

@Autonomous(name="Test Tracking Wheels", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestTrackingWheels extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        DecimalFormat df = new DecimalFormat("###.##");

        AbsoluteEncoder leftWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                centerWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                rightWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"));

        IncrementalAbsoluteEncoder leftIncremental = new IncrementalAbsoluteEncoder(leftWheel),
                centerIncremental = new IncrementalAbsoluteEncoder(centerWheel),
                rightIncremental = new IncrementalAbsoluteEncoder(rightWheel);

        while (!isStopRequested())
        {
            telemetry.addLine("Left: " + df.format(leftWheel.heading().degrees()));
            telemetry.addLine("Center: " + df.format(centerWheel.heading().degrees()));
            telemetry.addLine("Right: " + df.format(rightWheel.heading().degrees()));
            telemetry.addLine("~~~~~~~~~~~~~~~~");

            leftIncremental.updateIncremental();
            telemetry.addLine("Left Incremental: " + df.format(leftIncremental.getTotalAngularOffset()));

            centerIncremental.updateIncremental();
            telemetry.addLine("Center Incremental: " + df.format(centerIncremental.getTotalAngularOffset()));

            rightIncremental.updateIncremental();
            telemetry.addLine("Right Incremental: " + df.format(rightIncremental.getTotalAngularOffset()));

            telemetry.update();

            idle();
        }
    }
}

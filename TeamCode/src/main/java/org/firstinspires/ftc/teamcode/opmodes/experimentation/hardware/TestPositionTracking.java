package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.IncrementalAbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;

@Autonomous(name="Test Pose Tracking", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestPositionTracking extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        PoseTrackingEncoderWheelSystem ptews = new PoseTrackingEncoderWheelSystem(
                new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"))
        );

        while (!isStopRequested())
        {
            ptews.update();
            for (String line : ptews.getTrackingSummary())
                telemetry.addLine(line);
            telemetry.update();

            idle();
        }
    }
}

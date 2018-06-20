package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.IncrementalAbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;

import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.EnhancedOpMode;

@Autonomous(name="Test Pose Tracking", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestPositionTracking extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        PoseTrackingEncoderWheelSystem ptews = new PoseTrackingEncoderWheelSystem(
                new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"))
        );

        ProcessConsole console = log.newProcessConsole("Tracking Pose...");

        while (true)
        {
            ptews.update();

            flow.yield();
        }
    }
}

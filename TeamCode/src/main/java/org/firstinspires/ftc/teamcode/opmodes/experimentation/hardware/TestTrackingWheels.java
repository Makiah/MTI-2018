package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.IncrementalAbsoluteEncoder;

import java.text.DecimalFormat;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

@Autonomous(name="Test Tracking Wheels", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestTrackingWheels extends EnhancedOpMode
{
    @Override
    public void onRun() throws InterruptedException
    {
        DecimalFormat df = new DecimalFormat("###.##");

        AbsoluteEncoder leftWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                centerWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                rightWheel = new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"));

        leftWheel.setDirection(AbsoluteEncoder.Direction.REVERSE);

        IncrementalAbsoluteEncoder leftIncremental = new IncrementalAbsoluteEncoder(leftWheel),
                centerIncremental = new IncrementalAbsoluteEncoder(centerWheel),
                rightIncremental = new IncrementalAbsoluteEncoder(rightWheel);

        ProcessConsole console = log.newProcessConsole("Tracking Console");

        while (true)
        {
            leftIncremental.updateIncremental();
            centerIncremental.updateIncremental();
            rightIncremental.updateIncremental();

            console.write(
                    "Left: " + df.format(leftWheel.heading().degrees()),
                    "Center: " + df.format(centerWheel.heading().degrees()),
                    "Right: " + df.format(rightWheel.heading().degrees()),
                    "~~~~~~~~~~~~~~~~",
                    "Left velocity: " + df.format(leftIncremental.getCurrentAngularVelocity()),
                    "Center velocity: " + df.format(centerIncremental.getCurrentAngularVelocity()),
                    "Right velocity: " + df.format(rightIncremental.getCurrentAngularVelocity()),
                    "~~~~~~~~~~~~~~~~",
                    "Left Incremental: " + df.format(leftIncremental.getTotalDegreeOffset()),
                    "Center Incremental: " + df.format(centerIncremental.getTotalDegreeOffset()),
                    "Right Incremental: " + df.format(rightIncremental.getTotalDegreeOffset())
            );

            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 20));
        }
    }
}

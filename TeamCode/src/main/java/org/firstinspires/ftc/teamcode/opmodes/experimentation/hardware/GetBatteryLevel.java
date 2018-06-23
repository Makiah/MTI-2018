package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.ftccommon.FtcEventLoopModified;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.activity.FtcEventLoopHandlerModified;

@Autonomous(name="Get Battery Level", group= OpModeDisplayGroups.EXPERIMENTATION)
public class GetBatteryLevel extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        while (true)
        {
            log.lines("Battery level is " + FtcEventLoopHandlerModified.lastBatterySend);

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));
        }
    }
}

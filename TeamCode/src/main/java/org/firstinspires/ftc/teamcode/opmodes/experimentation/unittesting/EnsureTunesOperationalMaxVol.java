package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankutanku.EnhancedOpMode;
import hankutanku.music.Tunes;

@Autonomous(name="Force Max Vol", group="Experimentation")
public class EnsureTunesOperationalMaxVol extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Tunes.play(Tunes.Option.USSR_Anthem, true);

        while (Tunes.playing())
            flow.yield();
    }
}

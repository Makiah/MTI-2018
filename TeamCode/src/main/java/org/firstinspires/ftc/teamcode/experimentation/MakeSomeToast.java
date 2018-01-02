package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import hankextensions.RobotCore;

@Autonomous(name="Make Some Toast", group="Experimentation")
public class MakeSomeToast extends RobotCore
{
    protected void onRun() throws InterruptedException
    {
        // MAKATTACK
        AppUtil.getInstance().showToast(UILocation.BOTH, "It's ya boi... MAKATTACK");
        // MAKATTACK
    }
}

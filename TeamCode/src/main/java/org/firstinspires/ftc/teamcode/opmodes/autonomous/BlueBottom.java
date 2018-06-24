package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Bottom", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class BlueBottom extends Autonomous
{
    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public BalancePlate getBalancePlate() {
        return BalancePlate.BOTTOM;
    }

    @Override
    protected boolean dontAttemptGlyphs() {
        return false;
    }
}

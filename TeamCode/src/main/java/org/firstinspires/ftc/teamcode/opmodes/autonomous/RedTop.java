package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Top", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class RedTop extends Autonomous
{
    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public BalancePlate getBalancePlate() {
        return BalancePlate.TOP;
    }

    @Override
    protected boolean dontRunAuto() {
        return false;
    }
}

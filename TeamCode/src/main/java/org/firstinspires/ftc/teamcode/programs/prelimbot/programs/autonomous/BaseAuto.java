package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.autonomous;

import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;
import hankextensions.music.Tunes;

public abstract class BaseAuto extends HardwareBase {
    protected enum DIRECTION {
        BACKWARDS(1,1,0), LEFT(0,0,-1), FORWARD(-1,-1,0), RIGHT(0,0,1), STOP(0,0,0);

        public final double leftPower;
        public final double rightPower;
        public final double middlePower;

        DIRECTION(double leftPower, double rightPower, double middlePower) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.middlePower = middlePower;
        }
    }

    protected void drive(DIRECTION dir, long time) throws InterruptedException {
        drive(dir, time, 1.0);
    }

    protected void drive(DIRECTION dir, long time, double powerMul) throws InterruptedException {
        left.setPower(dir.leftPower);
        right.setPower(dir.rightPower);
        middle.setPower(dir.middlePower);
        Thread.sleep(time);
    }

    @Override
    protected void INITIALIZE() {
        closeClamps();
    }

    private void hailTheSoviets() {
        Tunes.play(Tunes.Option.USSR_Anthem);
    }
}

package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@TeleOp(name="Servo Testing", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class ServoTesting extends LinearOpMode
{
    /**
     * left (back robot perspective): port 0, up = .3, down is .9 (but they're reversed :/)
     * right port 1: .7 is up, .1 is down.
     *
     * Lower ramp grabber: unclamp = .178, clamp = .254 (port 3)
     * Upper ramp grabber: clamp = .301, unclamp = .217(port 2)
     *
     * blocker up = 0, block = .434(port 4)
     * Knocker: middle = .619, right = .087, left = 1.0(port 5)
     *
     * top block .32 is down .49 is up
     * @throws InterruptedException
     */

    @Override
    public void runOpMode() throws InterruptedException
    {
        final int numServos = 8;

        Servo[] servos = new Servo[numServos];

        for (int i = 0; i < numServos; i++)
            servos[i] = hardwareMap.servo.get("servo" + i);

        int current = 0;
        double currentPos = 0;

        while (!isStopRequested())
        {
            if (current >= numServos)
                current = 0;

            if (current < 0)
                current = numServos - 1;

            if (gamepad1.dpad_up)
                currentPos += .001;
            else if (gamepad1.dpad_down)
                currentPos -= .001;

            currentPos = Range.clip(currentPos, 0, 1);

            servos[current].setPosition(currentPos);

            telemetry.addLine("Position: " + currentPos);
            telemetry.addLine("Servo: " + current);
            telemetry.update();

            if (gamepad1.y)
            {
                current++;
                currentPos = 0;
                sleep(500);
            }

            if (gamepad1.x)
            {
                current--;
                currentPos = 0;
                sleep(500);
            }

            idle();
        }
    }
}

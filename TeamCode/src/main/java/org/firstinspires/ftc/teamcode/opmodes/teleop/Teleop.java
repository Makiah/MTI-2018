package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.EnhancedOpMode;
import hankutanku.input.HTButton;
import hankutanku.input.HTGamepad;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.Vector;

@TeleOp(name="Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class Teleop extends EnhancedOpMode
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, AutoOrTeleop.TELEOP, flow);

        while (!isStarted())
            flow.yield();

        boolean inFieldCentricMode = true;

        ProcessConsole console = log.newProcessConsole("Teleop");

        while (true)
        {
            HTGamepad.CONTROLLER1.update();
            // HTGamepad.CONTROLLER2.update(); (not currently in use)

            // Flipper
            if (HTGamepad.CONTROLLER1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.attemptFlipperStateIncrement();
            robot.flipper.updateFlipperState();

            // Lift
            if (gamepad1.right_bumper)
                robot.lift.setPower(1);
            else if (gamepad1.left_bumper)
                robot.lift.setPower(-1);
            else
                robot.lift.setPower(0);

            // Relic Arm
            if (gamepad1.dpad_left)
                robot.relic.setPower(1);
            else if (gamepad1.dpad_right)
                robot.relic.setPower(-1);
            else
                robot.relic.setPower(0);

            // Field Centric Toggle/Reset
            if (HTGamepad.CONTROLLER1.a.currentState == HTButton.ButtonState.JUST_TAPPED)
            {
                inFieldCentricMode = !inFieldCentricMode;

                if (inFieldCentricMode)
                    robot.ptews.reset();
            }

            Vector driveVector = HTGamepad.CONTROLLER1.leftJoystick();
            double turnSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;

            if (inFieldCentricMode)
            {
                robot.ptews.update();

                driveVector = driveVector.rotateBy(robot.ptews.getCurrentPose().heading.negative());

                if (HTGamepad.CONTROLLER1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.ptews.reset();
            }

            // Two different modes: intake and deposit.  The deposit mode results in the trigger being used to slow down the robot.
            if (robot.flipper.canIntakeGlyphs())
            {
                robot.harvester.run((gamepad1.left_trigger - gamepad1.right_trigger) * .7);
                robot.drivetrain.move(driveVector, turnSpeed);
            }
            else
            {
                robot.harvester.run(0);

                double powerReductionFactor = (3 * gamepad1.left_trigger + 1);
                robot.drivetrain.move(driveVector.divide(powerReductionFactor), turnSpeed / powerReductionFactor);
            }

            console.write("In " + (inFieldCentricMode ? "field" : "robot") + " centric mode");

            flow.yield();
        }
    }
}

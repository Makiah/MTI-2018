package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.vision.analysis.CryptoboxTracker;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTButton;
import hankextensions.vision.opencv.OpenCVCam;

public abstract class TeleopBase extends EnhancedOpMode implements CompetitionProgram
{
    private Robot robot;

    private ProcessConsole camConsole;
    private OpenCVCam cam;
    private CryptoboxTracker tracker;

    private boolean cryptoboxTestingEnabled = false;
    public void enableTestingCryptobox() throws InterruptedException
    {
        cam.start(tracker, true);
    }
    public void disableTestingCryptobox()
    {
        cam.stop();
    }
    public void toggleCryptoTesting() throws InterruptedException
    {
        cryptoboxTestingEnabled = !cryptoboxTestingEnabled;

        if (cryptoboxTestingEnabled)
            enableTestingCryptobox();
        else
            disableTestingCryptobox();
    }

    @Override
    protected final void onRun() throws InterruptedException
    {
        cam = new OpenCVCam();
        camConsole = log.newProcessConsole("Cam Console");
        tracker = new CryptoboxTracker();

        robot = new Robot(hardware);

        // Init robot hardware.
        robot.flipper.advanceStage(0);
        robot.intake.stop();

        // Init swerve drive for teleop
        robot.swerveDrive.setJoystickControlEnabled(true);
        //robot.swerveDrive.setAxleDrivingProtectionTo(true); // because our drivers need to chiiilll
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        waitForStart();

        while (true)
        {
            // Update controllers
            C1.update();
            C2.update();

            // Update swerve drive
            if (C1.b.currentState == HTButton.ButtonState.JUST_TAPPED)
            {
                if (robot.swerveDrive.getControlMethod() == SwerveDrive.ControlMethod.FIELD_CENTRIC)
                    robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.TANK_DRIVE);
                else
                    robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.FIELD_CENTRIC);
            }
            robot.swerveDrive.synchronousUpdate();

            // Control flipper
            if (C1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.advanceStage();

            if (C2.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.advanceStage();

            if (C1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.lights.toggleLights();

            // Control intake
            if (C1.gamepad.left_bumper)
                robot.intake.expel();
            else if (C1.gamepad.right_bumper)
                robot.intake.intake();
            else
                robot.intake.stop();

            // Control the lift.
            if (C2.gamepad.dpad_up)
                robot.lift.up();
            else if (C2.gamepad.dpad_down)
                robot.lift.down();
            else
                robot.lift.stop();

            // Controls the relic arm
            if (C2.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.relicSystem.toggleGrabber();

            robot.relicSystem.variableExtension(C2.gamepad.right_trigger, C2.gamepad.left_trigger);

            if (C2.gamepad.dpad_left)
                robot.relicSystem.rotate(false);
            else if (C2.gamepad.dpad_right)
                robot.relicSystem.rotate(true);
            else
                robot.relicSystem.stopRotator();

            // Temporary for cam testing
            if (C2.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                toggleCryptoTesting();

            if (cryptoboxTestingEnabled)
            {
                camConsole.write("Distances are " + tracker.placementDistances[0] + ", " + tracker.placementDistances[1] + " and " + tracker.placementDistances[2]);
            }

            flow.yield();
        }
    }
}

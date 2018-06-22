package hankutanku.activity;

import android.app.Activity;
import android.os.Bundle;

import com.acmerobotics.dashboard.RobotDashboard;

import dude.makiah.androidlib.threading.TaskParent;

import hankutanku.phonesensors.AndroidGyro;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;

/**
 * Neat little brainchild I had: why not have all vision code in the base class of FtcRobotControllerActivity
 * and just override it?
 */
public abstract class HankuBaseActivity extends Activity implements TaskParent
{
    public static HankuBaseActivity instance;
    public static boolean NO_DASHBOARD = false;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        instance = this;
        OpenCVCam.instance = null;
        VuforiaCam.instance = null;
        AndroidGyro.instance = null;

        // ACME's dashboard
        try
        {
            if (!NO_DASHBOARD)
                RobotDashboard.start(); // It freaks out when there's no receiver attached, so this stops it until the next run.
        }
        catch (Exception e)
        {
            e.printStackTrace();
            NO_DASHBOARD = true;
        } // Ignore

        // Enable when stable.
        Thread.setDefaultUncaughtExceptionHandler(new DefaultExceptionHandler(this, this));
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.RESUME);
    }

    @Override
    protected void onPause() {
        super.onPause();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.PAUSE);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);

//        if (OpenCVCam.instance != null)
//            OpenCVCam.instance.onWindowFocusChanged(hasFocus);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        if (!NO_DASHBOARD)
            RobotDashboard.stop();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.DESTROY);
    }

    @Override
    public boolean isTaskActive()
    {
        return true;
    }
}

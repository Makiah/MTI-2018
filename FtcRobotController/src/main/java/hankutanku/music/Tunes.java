package hankutanku.music;

import android.content.Context;
import android.media.AudioManager;
import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.R;

import dude.makiah.androidlib.logging.LoggingBase;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.activity.HankuBaseActivity;

/**
 * NiFTMusic enables the easy playing of audio files which are placed in the FtcRobotController/res/raw folder.  This functionality is helpful for both debugging and showing the other teams that you've got swag.
 */
public class Tunes
{
    /**
     * The enum which encapsulates the res/raw folder (make sure to pre-register songs here!)
     */
    public enum Option
    {
        USSR_Anthem (R.raw.ussranthem),
        LEFT_COL (R.raw.leftcol),
        CENTER_COL (R.raw.centercol),
        RIGHT_COL(R.raw.rightcol);

        public final int resourceAddress;
        Option(int resourceAddress)
        {
            this.resourceAddress = resourceAddress;
        }
    }

    private static MediaPlayer mediaPlayer = null;

    /**
     * Calling play initializes the media player with the given app context and starts playing a song.
     *
     * @param choice the pre-registered enum option which represents the file in question.
     */
    public static void play (Option choice, final boolean forceVolumeMax)
    {
        try
        {
            if (forceVolumeMax)
            {
                AudioManager am =
                        (AudioManager) HankuBaseActivity.instance.getSystemService(Context.AUDIO_SERVICE);

                am.setStreamVolume(
                        AudioManager.STREAM_MUSIC,
                        am.getStreamMaxVolume(AudioManager.STREAM_MUSIC),
                        0);
            }

            mediaPlayer = MediaPlayer.create (EnhancedOpMode.instance.hardwareMap.appContext, choice.resourceAddress);
            mediaPlayer.start ();
            mediaPlayer.setOnCompletionListener (new MediaPlayer.OnCompletionListener ()
            {
                public void onCompletion (MediaPlayer mediaPlayer1)
                {
                    mediaPlayer1.release ();

                    if (forceVolumeMax)
                    {
                        AudioManager am =
                                (AudioManager) HankuBaseActivity.instance.getSystemService(Context.AUDIO_SERVICE);

                        am.setStreamVolume(
                                AudioManager.STREAM_MUSIC,
                                1,
                                0);
                    }
                }
            });

            LoggingBase.instance.lines("Playing " + choice.toString ());

            EnhancedOpMode.instance.flow.pause (new TimeMeasure(TimeMeasure.Units.SECONDS, 1)); //Give the MediaPlayer some time to initialize, and register that a song is being played.
        } catch (InterruptedException e)
        {/**/} //Exit immediately.
        catch (Exception e)
        {
            EnhancedOpMode.instance.log.lines("Music error: " + e.getMessage ());
        }
    }

    public static void play(Option choice)
    {
        play(choice, false);
    }

    /**
     * Calling silence() stops the currently playing media player, if it is playing.
     */
    public static void silence()
    {
        try
        {
            if (mediaPlayer != null)
            {
                if (mediaPlayer.isPlaying ())
                    mediaPlayer.stop (); //stopEasyTask playing
                mediaPlayer.release (); //prevent resource allocation
                mediaPlayer = null; //nullify the reference.
            }
        }
        catch (IllegalStateException e)
        {}
    }

    public static boolean playing ()
    {
        return mediaPlayer != null;
    }
}

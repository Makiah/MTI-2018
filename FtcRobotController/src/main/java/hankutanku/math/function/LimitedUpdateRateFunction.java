package hankutanku.math.function;

import dude.makiah.androidlib.threading.TimeMeasure;

public interface LimitedUpdateRateFunction<T, U> extends Function<T, U>
{
    TimeMeasure getUpdateRate();
}

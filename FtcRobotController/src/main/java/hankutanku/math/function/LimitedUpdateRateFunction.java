package hankutanku.math.function;

import dude.makiah.androidlib.threading.TimeMeasure;

public interface LimitedUpdateRateFunction<T> extends Function<T>
{
    TimeMeasure getUpdateRate();
}

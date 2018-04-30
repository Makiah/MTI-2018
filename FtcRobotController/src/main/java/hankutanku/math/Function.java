package hankutanku.math;

/**
 * Can be a function of type Angle, type Double, etc.
 * @param <T>
 */
public interface Function<T>
{
    T value(double input);
}

package hankutanku.math.function;

/**
 * Can be a function of type Angle, type Double, etc.
 * @param <T>
 */
public interface Function<T, U>
{
    T value(U input);
}

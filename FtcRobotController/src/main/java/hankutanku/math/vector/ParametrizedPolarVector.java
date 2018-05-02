package hankutanku.math.vector;

import hankutanku.math.function.Function;
import hankutanku.math.angle.Angle;

/**
 * The parametrized polar vector.
 */
public class ParametrizedPolarVector implements Function<Vector>
{
    private final Function<Double> magnitude;
    private final Function<Angle> angle;

    public ParametrizedPolarVector(Function<Double> magnitude, Function<Angle> angle)
    {
        this.magnitude = magnitude;
        this.angle = angle;
    }

    public Vector value(double parameter)
    {
        return new PolarVector(magnitude.value(parameter), angle.value(parameter));
    }
}

package hankutanku.math.vector;

import hankutanku.math.function.Function;
import hankutanku.math.angle.Angle;

/**
 * The parametrized polar vector.
 */
public class ParametrizedPolarVector implements Function<Vector, Double>
{
    private final Function<Double, Double> magnitude;
    private final Function<Angle, Double> angle;

    public ParametrizedPolarVector(Function<Double, Double> magnitude, Function<Angle, Double> angle)
    {
        this.magnitude = magnitude;
        this.angle = angle;
    }

    public Vector value(Double parameter)
    {
        return new PolarVector(magnitude.value(parameter), angle.value(parameter));
    }
}

package hankutanku.math.vector;

import hankutanku.math.function.Function;

/**
 * A parametrized cartesian vector (rectangular coordinates
 */
public class ParametrizedCartesianVector implements Function<Vector, Double>
{
    private final Function<Double, Double> x, y;

    public ParametrizedCartesianVector(Function<Double, Double> x, Function<Double, Double> y)
    {
        this.x = x;
        this.y = y;
    }

    public Vector value(Double parameter)
    {
        return new CartesianVector(x.value(parameter), y.value(parameter));
    }
}

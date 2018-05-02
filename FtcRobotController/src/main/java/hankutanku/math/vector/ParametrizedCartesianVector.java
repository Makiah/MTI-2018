package hankutanku.math.vector;

import hankutanku.math.function.Function;

/**
 * A parametrized cartesian vector (rectangular coordinates
 */
public class ParametrizedCartesianVector implements Function<Vector>
{
    private final Function<Double> x, y;

    public ParametrizedCartesianVector(Function<Double> x, Function<Double> y)
    {
        this.x = x;
        this.y = y;
    }

    public Vector value(double parameter)
    {
        return new CartesianVector(x.value(parameter), y.value(parameter));
    }
}

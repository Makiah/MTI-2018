package hankutanku.math;

/**
 * Looks like multivariable calc was a useful class after all :P
 */
public class ParametrizedVector
{
    public static ParametrizedVector from(final Vector base)
    {
        return ParametrizedVector.rectangular(
                new Function<Double>()
                {
                    public Double value(double input)
                    {
                        return base.x;
                    }
                },
                new Function<Double>()
                {
                    public Double value(double input)
                    {
                        return base.y;
                    }
                });
    }

    public static ParametrizedVector polar(Function<Double> mag, Function<Double> theta)
    {
        return new ParametrizedVector(VariableVectorType.POLAR, mag, theta);
    }

    public static ParametrizedVector rectangular(Function<Double> x, Function<Double> y)
    {
        return new ParametrizedVector(VariableVectorType.RECTANGULAR, x, y);
    }

    // Type which this was initialized as.
    private enum VariableVectorType {POLAR, RECTANGULAR}
    private final VariableVectorType type;

    // The components of this function.
    private final Function<Double> a, b;

    private ParametrizedVector(VariableVectorType type, Function<Double> a, Function<Double> b)
    {
        this.type = type;

        this.a = a;
        this.b = b;
    }

    public Vector getVector(double param)
    {
        return type == VariableVectorType.POLAR ?
                Vector.polar(a.value(param), b.value(param)) :
                Vector.rectangular(a.value(param), b.value(param));
    }
}

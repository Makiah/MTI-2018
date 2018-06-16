package hankutanku.math.vfmp;

import hankutanku.math.function.Function;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

public class VectorField implements Function<Vector, Vector>
{
    private final Function<Double, Vector> i, j;

    public VectorField(Function<Double, Vector> i, Function<Double, Vector> j)
    {
        this.i = i;
        this.j = j;
    }

    @Override
    public Vector value(Vector input)
    {
        return new CartesianVector(i.value(input), j.value(input));
    }
}

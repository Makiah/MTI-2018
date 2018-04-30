package hankutanku.math.vector;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.angle.RadianAngle;

/**
 * Just has x and y calculated.
 */
public class CartesianVector extends Vector
{
    private final double x;
    @Override public double x()
    {
        return x;
    }

    private final double y;
    @Override public double y()
    {
        return y;
    }

    public CartesianVector(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    // Calculated upon request.
    private double magnitude = Double.NaN;
    @Override public double magnitude()
    {
        if (Double.isNaN(magnitude))
            magnitude = Math.hypot(x, y);

        return magnitude;
    }

    // Calculated upon request.
    private Angle angle = null;
    @Override public Angle angle()
    {
        if (angle == null)
            angle = new RadianAngle(Math.atan2(y, x));

        return angle;
    }
}

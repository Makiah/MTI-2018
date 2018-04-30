package hankutanku.math.vector;

import hankutanku.math.angle.Angle;

/**
 * Just has magnitude and theta calculated.
 */
public class PolarVector extends Vector
{
    private final double magnitude;
    @Override public double magnitude()
    {
        return magnitude;
    }

    private final Angle angle;
    @Override public Angle angle()
    {
        return angle;
    }

    public PolarVector(double magnitude, Angle angle)
    {
        this.magnitude = magnitude;
        this.angle = angle;
    }

    // Calculated upon request.
    private double x = Double.NaN;
    @Override public double x()
    {
        if (Double.isNaN(x))
            x = magnitude * Math.cos(Math.toRadians(angle.radians()));

        return x;
    }

    // Calculated upon request.
    private double y = Double.NaN;
    @Override public double y()
    {
        if (Double.isNaN(y))
            y = magnitude * Math.sin(Math.toRadians(angle.radians()));

        return y;
    }
}

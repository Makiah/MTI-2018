package hankutanku.math.angle;

/**
 * Just radians
 */
public class RadianAngle extends Angle
{
    private final double radians;
    @Override public double radians() { return radians; }

    public RadianAngle(double radians)
    {
        final double tau = 2 * Math.PI;
        while (radians >= tau)
            radians -= tau;
        while (radians < 0)
            radians += tau;

        this.radians = radians;
    }

    private double degrees = Double.NaN;
    @Override
    public double degrees()
    {
        if (Double.isNaN(degrees))
            degrees = Math.toDegrees(radians);

        return degrees;
    }
}

package hankutanku.math.angle;

/**
 * Just has degrees
 */
public class DegreeAngle extends Angle
{
    private final double degrees;
    @Override public double degrees() { return degrees; }

    public DegreeAngle(double degrees)
    {
        while (degrees >= 360)
            degrees -= 360;
        while (degrees < 0)
            degrees += 360;

        this.degrees = degrees;
    }

    private double radians = Double.NaN;
    @Override public double radians()
    {
        if (Double.isNaN(radians))
            radians = Math.toRadians(degrees);

        return radians;
    }
}

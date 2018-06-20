package hankutanku.math.angle;

public abstract class Angle
{
    public abstract double degrees();
    public abstract double radians();

    public Angle add(Angle other)
    {
        if (other instanceof DegreeAngle && this instanceof DegreeAngle)
            return new DegreeAngle(other.degrees() + this.degrees());

        return new RadianAngle(this.radians() + other.radians());
    }

    public Angle subtract(Angle other)
    {
        if (other instanceof DegreeAngle && this instanceof DegreeAngle)
            return new DegreeAngle(other.degrees() - this.degrees());

        return new RadianAngle(this.radians() - other.radians());
    }

    public Angle opposing()
    {
        if (this instanceof DegreeAngle)
            return new DegreeAngle(180 + degrees());
        else
            return new RadianAngle(Math.PI + radians());
    }

    public Angle negative()
    {
        if (this instanceof DegreeAngle)
            return new DegreeAngle(-degrees());
        else
            return new RadianAngle(-radians());
    }

    public double quickestDegreeMovementTo(Angle other)
    {
        double diff = (other.degrees() - degrees() + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
}

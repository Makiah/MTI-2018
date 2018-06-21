package hankutanku.math.vector;

import java.text.DecimalFormat;

import hankutanku.math.angle.Angle;

public abstract class Vector
{
    // Cartesian vector instance properties.
    public abstract double x();
    public abstract double y();

    // Polar vector instance properties.
    public abstract Angle angle();
    public abstract double magnitude();

    /**
     * Adds a vector to this.
     * @param other the other vector
     * @return the added result
     */
    public Vector add(Vector other)
    {
        return new CartesianVector(this.x() + other.x(), this.y() + other.y());
    }

    /**
     * Subtracts a vector from this.
     * @param other the other vector
     * @return the subtracted result
     */
    public Vector subtract(Vector other)
    {
        return new CartesianVector(this.x() - other.x(), this.y() - other.y());
    }

    /**
     * Multiplies a vector
     * @param coefficient the coefficient to multiply by
     * @return the multiplied vector
     */
    public Vector multiply(double coefficient)
    {
        if (this instanceof CartesianVector)
            return new CartesianVector(x() * coefficient, y() * coefficient);
        else
            return new PolarVector(magnitude() * coefficient, angle());
    }

    /**
     * Divides a vector.
     * @param coefficient the coefficient to divide by
     * @return the divided vector
     */
    public Vector divide(double coefficient)
    {
        return this.multiply(1.0 / coefficient);
    }

    /**
     * The unit vector for this vector.
     * @return the unit vector
     */
    public Vector unit()
    {
        return this.divide(magnitude());
    }

    /**
     * @param someAngle The angle to rotate by
     * @return the resulting angle.
     */
    public Vector rotateBy(Angle someAngle)
    {
        return new PolarVector(magnitude(), angle().add(someAngle));
    }

    /**
     * Does x * x + y * y if both are cartesian, otherwise |a||b|cos(theta)
     * @param other vector to dot by
     * @return the dot product
     */
    public double dot(Vector other)
    {
        if (this instanceof CartesianVector && other instanceof CartesianVector)
            return this.x() * other.x() + this.y() * other.y();
        else
            return this.magnitude() * other.magnitude() * Math.cos(Math.toRadians(this.angle().quickestDegreeMovementTo(other.angle())));
    }

    /**
     * Whether the vector is pretty much the same
     * @param other the other vector to check
     * @return whether they are the same
     */
    public boolean nearlyEquals(Vector other)
    {
        return (Math.abs(this.x() - other.x()) < .001) && (Math.abs(this.y() - other.y()) < .001);
    }

    // How we'll output the vector string.
    public static final DecimalFormat decimalFormat = new DecimalFormat("#.00");

    public String toString()
    {
        return toString(true);
    }
    public String toString(boolean usePolarFormat)
    {
        if (usePolarFormat)
            return "<" + decimalFormat.format(magnitude()) + ", " + decimalFormat.format(angle().degrees()) + " degrees>";
        else
            return "<" + decimalFormat.format(x()) + ", " + decimalFormat.format(y()) + ">";
    }
}

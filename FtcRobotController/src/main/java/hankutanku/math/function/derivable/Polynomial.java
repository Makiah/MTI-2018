package hankutanku.math.function.derivable;

public class Polynomial implements DerivableFunction
{
    private final int[] coefficients;

    public Polynomial(int[] coefficients)
    {
        this.coefficients = coefficients;
    }

    /**
     * if coefficients are [1, 2, 3, 4, 5], evaluated as 1*input^4 + 2*input^3 + 3*input^2 +
     * 4*input^1+5*input^0
     * @param input function input
     * @return
     */
    @Override
    public Double value(double input)
    {
        double cumulativeValue = 0;
        for (int i = 0; i < coefficients.length; i++)
            cumulativeValue += coefficients[i] * Math.pow(input, coefficients.length - 1 - i);

        return cumulativeValue;
    }

    /**
     * All this has to do is modify the coefficients array.
     * @return
     */
    public Polynomial deriveOnce()
    {
        int[] newCoefficients = new int[coefficients.length - 1];

        for (int i = 0; i < coefficients.length - 1; i++)
            newCoefficients[i] += coefficients[i] * (coefficients.length - 1 - i);

        return new Polynomial(newCoefficients);
    }

    @Override
    public Polynomial nthDerivative(int derivative)
    {
        Polynomial current = this;

        for (int i = 0; i < derivative; i++)
            current = current.deriveOnce();

        return current;
    }

    public String toString()
    {
        StringBuilder toReturn = new StringBuilder();
        for (int i = 0; i < coefficients.length; i++)
            toReturn.append(coefficients[i] + " * x^" + (coefficients.length - 1 - i) + " + ");

        return toReturn.toString();
    }
}

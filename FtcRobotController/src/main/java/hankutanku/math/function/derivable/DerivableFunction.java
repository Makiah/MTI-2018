package hankutanku.math.function.derivable;

import hankutanku.math.function.Function;

public interface DerivableFunction extends Function<Double>
{
    DerivableFunction nthDerivative(int derivative);
}

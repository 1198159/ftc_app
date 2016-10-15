package org.firstinspires.ftc.team6220;

/*
    Finite Impulse Response Filter
    Calculates a weighted average based on the age of input values.
    Used to prevent or induce certain output frequencies

    INPUTS:
      -Control value (e.g. joystick value)

    OUTPUTS:
      -Filtered value (e.g. a smoother, non-step function)
*/

public class FIRFilter implements Filter
{
    public double[] values;
    private double[] weights;

    public FIRFilter(double[] w)
    {
        weights = w;
    }

    //initalize with weight as a polynominal function of age
    public FIRFilter(Polynomial func,int depth)
    {
        weights = SequenceUtilities.arrayFromPolynomial(0,depth,func);
    }

    //setup with new value set
    public void roll(double newValue)
    {
        SequenceUtilities.roll(values,newValue);
    }

    public double getFilteredValue()
    {
        return SequenceUtilities.weightedAverage(values, weights);
    }
}

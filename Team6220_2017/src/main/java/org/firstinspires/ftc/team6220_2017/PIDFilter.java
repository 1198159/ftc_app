package org.firstinspires.ftc.team6220_2017;

 /*
    Proportional-Integral-Derivative Filter
    Calculates a input difference, it's first integral, and first derivative.
    Generally used to produce efficient, non-oscillating motion in a one dimensional system.

    INPUTS:
      -(once) PID coefficients (tuning values)
      -Difference between the target value and current value. (e.g a target angle vs encoder position)

    OUTPUTS:
      -First derivative control. (e.g. a motor power)
*/

//TODO decide if this should handle the PID enforcement mode
public class PIDFilter implements Filter
{
    //CodeReview: while cute, and legal, using greek characters in your code makes it harder for others to work with this code
    //           (they may not know how to type those chars, and/or may not understand what they mean).
    //           You would be better off making these less obscure by using descriptive names.

    //Proportional coefficient
    private double εP;
    //Integral coefficient
    private double εI;
    //Derivative coefficient
    private double εD;

    //construct with the coefficients
    public PIDFilter(double P, double I, double D)
    {
        εP = P;
        εI = I;
        εD = D;
    }

    public double[] values = new double[2];
    public double sum = 0;
    public double dV  = 0;


    //update with new value
    public void roll(double newValue)
    {
        //update calculated values
        sum += values[0];
        dV = values[0]-values[1];

        //introduce new value
        values[1] = values[0];
        values[0] = newValue;
    }

    public double getFilteredValue()
    {
        return (εP*values[0] ) + ( εI* sum) + ( εD*dV );
    }

}
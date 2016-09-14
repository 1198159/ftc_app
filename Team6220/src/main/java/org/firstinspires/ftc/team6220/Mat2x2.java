package org.firstinspires.ftc.team6220;

/*
    2 by 2 Matrix.
    Used to transform 2D vectors
*/

public class Mat2x2
{
    private double[][] values = null;
    public static double[][] identity = {  {1.0, 0.0},
            {0.0, 1.0}  };
    //construct as identity
    public Mat2x2()
    {
        this.values = Mat2x2.identity;
    }

    //construct with an array
    public Mat2x2(double[][] arr)
    {
        this.values = arr;
    }

    //construct with an angle
    public Mat2x2(double w)
    {
        this.values[0][0] = Math.sin(w);
        this.values[0][1] = this.values[0][0];
        this.values[1][1] = Math.cos(w);
        this.values[1][0] = this.values[0][1];
    }

    //get
    public double get(int x, int y)
    {
        return this.values[x][y];
    }
}

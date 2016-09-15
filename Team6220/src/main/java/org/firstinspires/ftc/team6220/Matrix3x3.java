package org.firstinspires.ftc.team6220;

/*
    3 by 3 Matrix.
    Used to transform 2D vectors.
*/

import static  java.lang.Math.*;

public class Matrix3x3 extends Matrix
{

    public double[][] values;

    public static double[][] identity = { { 1 , 0 , 0 },
                                          { 0 , 1 , 0 },
                                          { 0 , 0 , 1 } };
    public static double[][] empty    = { { 0 , 0 , 0 },
                                          { 0 , 0 , 0 },
                                          { 0 , 0 , 0 } };


    //construct a matrix from a translation and rotation transformation
    public Matrix3x3(double x, double y, double w)
    {
        this.values = new double[][]{   { cos(w), -sin(w), x  },
                                   { sin(w),  cos(w), y  },
                                   { 0     ,  0     , 1  } };
    }
    //construct a matrix from a translation vector and rotation transformation
    public Matrix3x3(Vector2D v, double w)
    {
        this.values = new Matrix3x3(v.values[0],v.values[1], w).values;
    }


    //multiply this matrix by another, resulting in a combined transformation
    public Matrix3x3 multiplied(Matrix3x3 mat)
    {
        return (Matrix3x3) this.multiplied(mat);
    }

}

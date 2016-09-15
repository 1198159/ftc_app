package org.firstinspires.ftc.team6220;

/*
    4 by 4 Matrix.
    Used to transform 3D vectors.
*/

import static java.lang.Math.*;

public class Mat4x4 extends Matrix
{
    public double[][] values;

    public static double[][] identity = { { 1 , 0 , 0 , 0 },
                                          { 0 , 1 , 0 , 0 },
                                          { 0 , 0 , 1 , 0 },
                                          { 0 , 0 , 0 , 1 } };
    public static double[][] empty    = { { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 } };


    //return a matrix from an euler rotation with any order
    public Mat4x4(RotationOrder order, double first, double second, double third)
    {
        Mat4x4 heading  = new Mat4x4(order.firstRot , first);
        Mat4x4 altitude = new Mat4x4(order.secondRot, second);
        Mat4x4 bank     = new Mat4x4(order.thirdRot , third);

        this.values = heading.multiplied(altitude).multiplied(bank).values;
    }
    //construct a matrix from a translation and rotation
    //rotation is about vector v of amount w
    //translation is to vector u
    //rotates, then moves
    public Mat4x4(Vector3D u, Vector3D v, double w)
    {
        double C = cos(w);
        double S = sin(w);
        double x = v.values[0];double y = v.values[1];double z = v.values[2];

        this.values = new double[][]{   { x*x*(1-C) +   C , y*x*(1-C) - z*S , z*x*(1-C) + y*S , u.values[0] },
                                        { x*y*(1-C) + z*S , y*y*(1-C) +   S , z*y*(1-C) - x*S , u.values[1] },
                                        { x*z*(1-C) - y*S , y*z*(1-C) + x*S , z*z*(1-C) +   S , u.values[2] },
                                        {      0          ,      0          ,      0          ,     1       } };
    }
    //construct a matrix from a rotation
    //rotation is about vector v of amount w
    public Mat4x4(Vector3D v, double w)
    {
        this.values = new Mat4x4(Vector3D.zeroVector,v,w).values;
    }
    //construct a matrix from a translation
    public Mat4x4(Vector3D v)
    {
        this.values = new Mat4x4(v,Vector3D.zeroVector,0.0).values;
    }

    //multiply this matrix by another, resulting in a combined transformation
    public Mat4x4 multiplied(Mat4x4 mat)
    {
        return (Mat4x4)this.multiplied(mat);
    }

}

package org.firstinspires.ftc.team6220;

/*
    4 by 4 Matrix.
    Used to transform 3D vectors.
*/

import static java.lang.Math.*;

public class Matrix4x4
{
    public double[][] values = new double[4][4];

    public static double[][] identity = { { 1 , 0 , 0 , 0 },
                                          { 0 , 1 , 0 , 0 },
                                          { 0 , 0 , 1 , 0 },
                                          { 0 , 0 , 0 , 1 } };
    public static double[][] empty    = { { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 },
                                          { 0 , 0 , 0 , 0 } };

    //construct a matrix identity
    public Matrix4x4()
    {
        this.values = empty;
    }
    //construct a matrix from an array
    public Matrix4x4(double[][] array)
    {
        this.values = array;
    }
    //return a matrix from an euler rotation with any order
    public Matrix4x4(RotationOrder order, double first, double second, double third)
    {
        Matrix4x4 heading  = new Matrix4x4(order.firstRot , first);
        Matrix4x4 altitude = new Matrix4x4(order.secondRot, second);
        Matrix4x4 bank     = new Matrix4x4(order.thirdRot , third);

        this.values = heading.multiplied(altitude).multiplied(bank).values;
    }
    //construct a matrix from a translation and rotation
    //rotation is about vector v of amount w
    //translation is to vector u
    //rotates, then moves
    public Matrix4x4(Vector3D pos, Vector3D v, double rot)
    {
        double C = cos(rot);
        double S = sin(rot);

        double[][] newValues =          { { v.x*v.x*(1-C) +     C , v.y*v.x*(1-C) - v.z*S , v.z*v.x*(1-C) + v.y*S , pos.x    },
                                        { v.x*v.y*(1-C)   + v.z*S , v.y*v.y*(1-C) +     S , v.z*v.y*(1-C) - v.x*S , pos.y    },
                                        { v.x*v.z*(1-C)   - v.y*S , v.y*v.z*(1-C) + v.x*S , v.z*v.z*(1-C) +     S , pos.z    },
                                        {          0              ,          0            ,          0            ,     1    } };
        this.values = newValues;
    }
    //construct a matrix from a rotation
    //rotation is about vector v of amount w
    public Matrix4x4(Vector3D v, double w)
    {
        this.values = new Matrix4x4(Vector3D.zeroVector,v,w).values;
    }
    //construct a matrix from a translation
    public Matrix4x4(Vector3D v)
    {
        this.values = new Matrix4x4(v,Vector3D.zeroVector,0.0).values;
    }


    //return the product of two matrices
    public Matrix4x4 multiplied(Matrix4x4 matB)
    {
        Matrix4x4 matA = new Matrix4x4(this.values);
        double[][] newValues = Matrix4x4.empty;
        for (int a = 0; a < 4;a++)//for each output row
        {
            for (int b = 0; b < 4; b++ )//for each output column
            {
                for(int c = 0; c < 4; c++)
                {
                     newValues[a][b] += matA.values[a][c]*matB.values[c][b];
                }
            }
        }
        return new Matrix4x4(newValues);
    }
    //multiply this matrix by another
    public void multiply(Matrix4x4 matB)
    {
        this.values = this.multiplied(matB).values;
    }

    //return the transpose of this matrix
    public Matrix4x4 transposed()
    {
        double[][] newValues = Matrix4x4.empty;
        int size = this.values[0].length;
        for (int a = 0; a < size;a++)
        {
            for (int b = 0; b < size; b++ )
            {
                newValues[a][b] = values[b][a];
            }
        }
        return new Matrix4x4(newValues);
    }
    //transpose this matrix
    public void transpose()
    {
        this.values = this.transposed().values;
    }


    //calculate the determinate of this matrix
    public double getDeterminate()
    {
        double det = 0.0;
        int size = this.values[0].length;
        //iterate through each diagonal
        for (int a = 0; a < size;a++)
        {
            for (int b = 0; b < size; b++ )
            {
                det += this.values[(a+b)%size][b];
            }
        }
        return det;
    }


    //calculate the inverse of this matrix
    public Matrix4x4 inverse()
    {
        double invDet = 1/this.getDeterminate();
        Matrix4x4 t = this.transposed();
        int size = this.values[0].length;
        for (int a = 0; a < size;a++)
        {
            for (int b = 0; b < size; b++ )
            {
                t.values[a][b] *= invDet;
            }
        }
        return t;
    }
    //set this matrix as its inverse
    public void invert()
    {
        this.values = this.inverse().values;
    }

}


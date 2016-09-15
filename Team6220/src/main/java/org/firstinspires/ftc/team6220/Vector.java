package org.firstinspires.ftc.team6220;

import static java.lang.Math.*;

/*
    Contains basic methods associated with N dimension vectors.
    NOTE: "java.util.Vector" is not a normal math vector, which is why we are not using it.
*/

public class Vector
{
    public double[] values;

    public static Vector zeroVector;


    //construct an empty vector
    public Vector()
    {
        this.values = zeroVector.values;
    }
    //construct with an array
    public Vector(double[] array)
    {
        this.values = array;
    }


    //get the length of this vector
    public double getMagnitude()
    {
        double sum = 0.0;
        int size = this.values.length;
        for (int a =0; a < size-1; a++)
        {
            sum += pow(this.values[a],2);
        }
        return Math.sqrt(sum);
    }


    //return a unit length vector in the same direction
    public Vector normalized()
    {
        Vector newVector = new Vector();
        double mag = this.getMagnitude();
        int size = this.values.length;
        for (int a =0; a < size-1; a++)
        {
            newVector.values[a] = this.values[a]/mag;
        }
        return newVector;
    }
    //set this vector as unit length
    public void normalize()
    {
        this.values = this.normalized().values;
    }


    //return the sum of this and another vector
    public Vector addedTo(Vector vectB)
    {
        Vector vectorA = new Vector(this.values);
        int size = this.values.length;
        for (int a =0; a < size-1; a++)
        {
            vectorA.values[a] += vectB.values[a];
        }

        return vectorA;
    }
    //add/move this vector by another
    public void add(Vector vectB)
    {
        this.values = this.addedTo(vectB).values;
    }


    //return a transformed vector by a matrix
    public Vector matrixMultiplied(Matrix m)
    {
        Vector newVector = Vector.zeroVector;
        int size = this.values.length;
        for(int a=0; a < size; a++)
        {
            for(int b=0; b < size; b++)
            {
                newVector.values[a] += this.values[b] * m.values[a][b];
            }
        }
        return newVector;
    }
    //multiply this vector by a matrix, transforming it
    public void matrixMultiply(Matrix m)
    {
        this.values = this.matrixMultiplied(m).values;
    }


    //return a multiplied vector with a non-uniform scaling
    //e.g return [ x1*x2    y1*y2    z1*z2  ]
    public Vector scalarMultiplied(Vector vectB)
    {
        Vector newVector = Vector.zeroVector;
        for(int a=0; a < 4; a++)
        {
            newVector.values[a] += this.values[a] * vectB.values[a];
        }
        return newVector;
    }
    //scale this vector by a non-uniform scaling
    public void scalarMultiply(Vector vectB)
    {
        this.values = this.scalarMultiplied(vectB).values;
    }

}

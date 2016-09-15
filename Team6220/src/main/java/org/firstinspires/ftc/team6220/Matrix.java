package org.firstinspires.ftc.team6220;

public class Matrix
{
    public double[][] values;

    public static double[][] identity;
    public static double[][] empty;


    //construct a matrix identity
    public Matrix()
    {
        values = empty;
    }
    //construct a matrix from an array
    public Matrix(double[][] array)
    {
        values = array;
    }


    //return the product of two matrices
    public Matrix multiplied(Matrix matB)
    {
        Matrix matA = new Matrix(this.values);
        double[][] newValues = Matrix.empty;
        int size = matA.values[0].length;
        for (int a = 0; a < size;a++)//for each output row
        {
            for (int b = 0; b < size; b++ )//for each output column
            {
                for(int c = 0; c < size; c++)
                {
                     newValues[a][b] += matA.values[a][c]*matB.values[c][b];
                }
            }
        }
        return new Matrix(newValues);
    }
    //multiply this matrix by another
    public void multiply(Matrix matB)
    {
        this.values = this.multiplied(matB).values;
    }

    //return the transpose of this matrix
    public Matrix transposed()
    {
        double[][] newValues = Matrix.empty;
        int size = this.values[0].length;
        for (int a = 0; a < size;a++)
        {
            for (int b = 0; b < size; b++ )
            {
                newValues[a][b] = values[b][a];
            }
        }
        return new Matrix(newValues);
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
    public Matrix inverse()
    {
        double invDet = 1/this.getDeterminate();
        Matrix t = this.transposed();
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

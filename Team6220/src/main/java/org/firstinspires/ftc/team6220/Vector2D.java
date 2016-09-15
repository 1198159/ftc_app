package org.firstinspires.ftc.team6220;

/*
    Represents a direction and magnitude in 2D.
    Defines points or directions in 2D.
*/

//TODO move to a higher scope
public class Vector2D extends Vector
{
    //a 2d vector with 3 depth is intentional to maintain compatibility with affine transforms
    //see: https://en.wikipedia.org/wiki/Transformation_matrix
    public double[] values = new double[] { 0.0, 0.0, 1.0 };

    //unit cardinal vectors
    public static Vector2D xAxis      = new Vector2D(1.0,0.0);
    public static Vector2D yAxis      = new Vector2D(0.0,1.0);
    public static Vector2D zeroVector = new Vector2D(0.0,0.0);


    //construct with an xy pair
    public Vector2D(double x, double y)
    {
        this.values[0] = x;
        this.values[1] = y;
    }


    //return the sum of this vector and an x and y value
    public Vector2D addedTo(double x, double y)
    {
        Vector2D vectB = new Vector2D(x,y);
        return (Vector2D)this.addedTo(vectB);
    }
    //move this vector by x and y
    public void add(double x, double y)
    {
        this.values = this.addedTo(x,y).values;
    }


    //calculate the heading of this vector
    public double getAngle()
    {
        return Math.atan2(this.values[1],this.values[0]);
    }
}


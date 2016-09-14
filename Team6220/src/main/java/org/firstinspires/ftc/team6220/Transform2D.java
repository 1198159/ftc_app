package org.firstinspires.ftc.team6220;

/*
    Type for holding position and orientation data for spaces and objects.
*/

//TODO move to a higher scope
public class Transform2D extends Vector2D
{
    //w represents yaw, the only orientation value in 2D space
    public double w;

    //construct from array
    public Transform2D(double[] values)
    {
        this.x = values[0];
        this.y = values[1];
        this.w = values[2];
    }

    //construct from individual values
    public Transform2D(double xI, double yI, double wI)
    {
        this.x = xI;
        this.y = yI;
        this.w = wI;
    }

    //construct from a vector and orientation
    public Transform2D(Vector2D v, double wI)
    {
        this.x = v.x;
        this.y = v.y;
        this.w = wI;
    }

    //get the orientation of this transform as a matrix
    public Mat2x2 getMatrix()
    {
        return new Mat2x2(this.w);
    }

    //get the location of this transform as a vector
    public Vector2D getPositionVector()
    {
        return new Vector2D(this.x,this.y);
    }
}


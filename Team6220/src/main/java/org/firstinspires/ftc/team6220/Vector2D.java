package org.firstinspires.ftc.team6220;

/*
    Represents a direction and magnitude in 2D.
    Defines points or directions in 2D.
*/

//TODO move to a higher scope
//TODO add Vector3D
//TODO add normalize()
public class Vector2D
{
    public double x;
    public double y;

    //construct empty
    public Vector2D()
    {
        this.x = 0.0;
        this.y = 0.0;
    }

    //construct with an x,y array
    public Vector2D(double[] values)
    {
        this.x = values[0];
        this.y = values[1];
    }

    //construct with either an xy pair
    public Vector2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    //get the length of this vector
    public double getMagnitude()
    {
        double sum = Math.pow(this.x,2) + Math.pow(this.y,2);
        return Math.sqrt(sum);
    }

    //get the pointing direction of the vector
    public double toAngle()
    {
        return Math.atan2(y, x);
    }

    //move this vector by another vector
    public void add(Vector2D vector)
    {
        this.add(vector.x,vector.y);
    }

    //move this vector by an x and y value
    public void add(double u, double v)
    {
        this.x += u;
        this.y += v;
    }

    //rotate this vector by a matrix
    public void matrixMultiply(Mat2x2 mat)
    {
        double xNew = this.x*mat.get(0,0) + this.y*mat.get(0,1);
        double yNew = this.x*mat.get(1,0) + this.y*mat.get(1,1);
        this.x = xNew;
        this.y = yNew;
    }

    //move and rotate this vector according to a transform
    public void transformTo(Transform2D t)
    {
        Mat2x2 mat = t.getMatrix();
        this.matrixMultiply(mat);
        this.add(t.getPositionVector());
    }

}


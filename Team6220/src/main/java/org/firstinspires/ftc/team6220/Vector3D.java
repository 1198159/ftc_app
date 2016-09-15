package org.firstinspires.ftc.team6220;

/*
    Represents a direction and magnitude in 3D.
    Defines points or directions in 3D.
*/

public class Vector3D extends Vector
{
    //a 3d vector with 4 depth is intentional to maintain compatibility with affine transforms
    //see: https://en.wikipedia.org/wiki/Transformation_matrix
    public double[] values = new double[] { 0.0, 0.0, 0.0, 1.0 };

    //unit cardinal vectors
    public static Vector3D xAxis      = new Vector3D( 1 , 0 , 0 );
    public static Vector3D yAxis      = new Vector3D( 0 , 1 , 0 );
    public static Vector3D zAxis      = new Vector3D( 0 , 0 , 1 );
    public static Vector3D zeroVector = new Vector3D( 0 , 0 , 0 );


    //construct with xyz values
    public Vector3D(double x, double y, double z)
    {
        this.values[0] = x;
        this.values[1] = y;
        this.values[2] = z;
    }


    //return the sum of this vector and an x, y and z value
    public Vector3D addedTo(double x, double y, double z)
    {
        Vector3D vectB = new Vector3D(x,y,z);
        return (Vector3D)this.addedTo(vectB);
    }
    //move this vector by x and y
    public void add(double x, double y, double z)
    {
        this.values = this.addedTo(x,y,z).values;
    }

}

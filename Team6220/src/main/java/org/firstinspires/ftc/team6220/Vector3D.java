package org.firstinspires.ftc.team6220;

/*
    Represents a direction and magnitude in 3D.
    Defines points or directions in 3D.
*/

public class Vector3D
{
    //a 3d vector with 4 depth is intentional to maintain compatibility with affine transforms
    //see: https://en.wikipedia.org/wiki/Transformation_matrix
    public double x = 0.0;
    public double y = 0.0;
    public double z = 0.0;
    public double w = 1.0;


    //unit cardinal vectors
    public static Vector3D xAxis      = new Vector3D( 1 , 0 , 0 , 0);
    public static Vector3D yAxis      = new Vector3D( 0 , 1 , 0 , 0);
    public static Vector3D zAxis      = new Vector3D( 0 , 0 , 1 , 0);
    public static Vector3D zeroVector = new Vector3D( 0 , 0 , 0 , 0);


    //construct with an array
    public Vector3D(double[] array)
    {
        this.x = array[0];
        this.y = array[1];
        this.z = array[2];
        this.w = array[3];
    }
    //construct with xyz values
    public Vector3D(double xValue, double yValue, double zValue)
    {
        this.x = xValue;
        this.y = yValue;
        this.z = zValue;
    }
    //construct with xyzw values
    public Vector3D(double xValue, double yValue, double zValue, double wValue)
    {
        this.x = xValue;
        this.y = yValue;
        this.z = zValue;
        this.w = wValue;
    }

    //return the sum of this vector and an x, y and z value
    public Vector3D addedTo(Vector3D vectB)
    {
        double[] newValues = {this.x,this.y,this.z};
        newValues[0] += vectB.x;
        newValues[1] += vectB.y;
        newValues[2] += vectB.z;
        return new Vector3D(newValues);
    }
    //return the sum of this vector and an x, y and z value
    public Vector3D addedTo(double xValue, double yValue, double zValue)
    {
        Vector3D vectB = new Vector3D(xValue,yValue,zValue);
        return (Vector3D)this.addedTo(vectB);
    }
    //move this vector by x,y, and z
    public void add(double x, double y, double z)
    {
        double[] newValues = this.addedTo(x,y,z).asArray();
        this.x = newValues[0];
        this.y = newValues[1];
        this.z = newValues[2];
    }

    //return an array that holds this vector's xyz values
    public double[] asArray()
    {
        return new double[] {this.x,this.y,this.z,this.w};
    }

    //get the length of this vector
    public double getMagnitude()
    {
        double sum = 0.0;
        for (int a =0; a < 3; a++)
        {
            sum += Math.pow(this.asArray()[a], 2);
        }
        return Math.sqrt(sum);
    }


    //return a unit length vector in the same direction
    public Vector3D normalized()
    {
        double[] newVector = new double[] {0.0,0.0,0.0,0.0};
        double mag = this.getMagnitude();
        for (int a =0; a < 3; a++)
        {
            newVector[a] = this.asArray()[a]/mag;
        }
        return new Vector3D(newVector);
    }
    //set this vector as unit length
    public void normalize()
    {
        Vector3D newVector = this.normalized();
        this.x = newVector.x;
        this.y = newVector.y;
        this.z = newVector.z;
        this.w = 0.0;
    }


    //return a transformed vector by a matrix
    public Vector3D matrixMultiplied(Matrix4x4 m)
    {
        double[] newVector = new double[] {0.0,0.0,0.0,0.0};
        for(int a=0; a < 4; a++)
        {
            for(int b=0; b < 4; b++)
            {
                newVector[a] += this.asArray()[b] * m.values[a][b];
            }
        }
        return new Vector3D(newVector);
    }
    //multiply this vector by a matrix, transforming it
    public void matrixMultiply(Matrix4x4 m)
    {
        Vector3D newVector = this.matrixMultiplied(m);
        this.x = newVector.x;
        this.y = newVector.y;
        this.z = newVector.z;
    }

}

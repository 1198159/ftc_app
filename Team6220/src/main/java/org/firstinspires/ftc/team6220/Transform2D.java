package org.firstinspires.ftc.team6220;

/*

*/
public class Transform2D
{
    public double x;
    public double y;
    public double rot;

    //construct a transform with a xy position and a rotation
    public Transform2D(double x, double y, double r)
    {
        this.x = x;
        this.y = y;
        this.rot = r;
    }

    //return the position vector of this tranformation
    public Vector3D getPositionVector()
    {
        return new Vector3D(this.x,this.y, 0.0);
    }

    //return a xy plane matrix that is this transformation
    public Matrix4x4 getMatrix()
    {
        Vector3D position = this.getPositionVector();
        return new Matrix4x4(position,Vector3D.zAxis,this.rot);
    }
    //return a xy plane matrix that is the inverse of this transformation
    public Matrix4x4 getInverseMatrix()
    {
        return this.getMatrix().inverse();
    }
}

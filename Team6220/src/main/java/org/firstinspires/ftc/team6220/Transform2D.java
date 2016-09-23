package org.firstinspires.ftc.team6220;

import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/*

*/
public class Transform2D
{
    //in meters
    public double x;
    public double y;
    //in degrees
    public double rot;

    //construct a location with a xy position and a rotation
    public Transform2D(double x, double y, double r)
    {
        this.x = x;
        this.y = y;
        this.rot = r;
    }

    //return the position vector of this transformation
    public VectorF getPositionVector()
    {
        return new VectorF((float)this.x, (float)this.y, 0);
    }

    //return a xy plane matrix that is this transformation
    public OpenGLMatrix getMatrix()
    {
        //translation to x,y,0.0
        OpenGLMatrix position = OpenGLMatrix.translation((float)this.x,(float)this.y,0);
        //rotation about z axis
        OpenGLMatrix rotation = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, (float)this.rot);
        //combined as rotate, then move
        return rotation.multiplied(position);
    }
    //return an xy plane matrix that is the inverse of this transformation
    public OpenGLMatrix getInverseMatrix()
    {
        return this.getMatrix().inverted();
    }
}

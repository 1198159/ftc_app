package org.firstinspires.ftc.team6220;

/*
    Contains different orders in which rotations can be represented.
    This is only relevant in n>2 dimensions.
*/

public class RotationOrder
{
    public Vector3D firstRot;
    public Vector3D secondRot;
    public Vector3D thirdRot;

    //order in which vuforia represents its output values
    public static RotationOrder vuforiaEulerOrder = new RotationOrder(Vector3D.xAxis, Vector3D.yAxis, Vector3D.zAxis);

    public RotationOrder(Vector3D a, Vector3D b, Vector3D c)
    {
        this.firstRot = a;
        this.secondRot= b;
        this.thirdRot = c;
    }
}

package org.firstinspires.ftc.team6220;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.HashMap;

/*
    Handles data output from vuforia and allows the user to see high-level information about it.

    INPUTS:
      -Vuforia data hashmap from .getVuforiaData()
      -(once) Target string index, e.g. "lego"
      -(optional)(once) Matrix4x4 representing camera location relative to robot control point, if omitted camera is assumed to be at robot origin
      -(optional)(once) Matrix4x4 representing target field location, if omitted camera is assumed to be at field origin

    OUTPUTS:
      -Target Location and orientation relative to robot
      -Robot Location and orientation relative to target or field
      -Estimated location and orientation error
*/
public class TrackInterpreter
{
    public HashMap<String,double[]> vData;
    public String targetName;
    public OpenGLMatrix robotToCamera;
    public OpenGLMatrix fieldToTarget;

}

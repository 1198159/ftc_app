package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.VuforiaTracker;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;

/*
    Test opmode that displays one value from a tracked object from vuforia.
*/

@Autonomous(name="Vuforia Test", group="Tests")
public class VuforiaTest extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException
    {
        //set up vuforia
        VuforiaTracker Tracker = FtcRobotControllerActivity.getVuforia();
        Tracker.addTrackables("BeaconImages.xml");
        Tracker.initVuforia();

        waitForStart();

        while(opModeIsActive())
        {
            //index data by tracked object
            HashMap<String, double[]> data = Tracker.getVuforiaData();

            if(data.containsKey("lego"))
            {
                try
                {
                    float rX = (float)data.get("lego")[0];
                    float rY = (float)data.get("lego")[1];
                    float rZ = (float)data.get("lego")[2];
                    float pX = (float)data.get("lego")[3];
                    float pY = (float)data.get("lego")[4];
                    float pZ = (float)data.get("lego")[5];
                    VectorF camRelTargetPos = new VectorF(pX,pY,pZ);
                    OpenGLMatrix cameraToTarget = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, rX, rY, rZ);
                    telemetry.addData("Pitch:", Math.toDegrees(rX));
                    telemetry.addData("Yaw:", Math.toDegrees(rY));
                    telemetry.addData("Dist:", camRelTargetPos.length());
                    telemetry.addData("locY:", cameraToTarget.multiplied(camRelTargetPos).get(1));

                }
                catch(Exception e)
                {
                    telemetry.addData("FAIL", e);
                }

            }

            telemetry.update();
        }

        try {
            Tracker.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}

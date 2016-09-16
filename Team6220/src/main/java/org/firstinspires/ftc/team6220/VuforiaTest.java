package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.VuforiaTracker;

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

            //
            if(data.containsKey("lego"))
            {
                try
                {
                    double rX = data.get("lego")[0];
                    double rY = data.get("lego")[1];
                    double rZ = data.get("lego")[2];
                    double pX = data.get("lego")[3];
                    double pY = data.get("lego")[4];
                    double pZ = data.get("lego")[5];
                    Vector3D camRelTargetPos = new Vector3D(pX,pY,pZ);
                    Matrix4x4 cameraToTarget = new Matrix4x4(RotationOrder.vuforiaEulerOrder,rX,rY,rZ);
                    telemetry.addData("Pitch:", Math.toDegrees(rX));
                    telemetry.addData("Yaw:", Math.toDegrees(rY));
                    telemetry.addData("Dist:", camRelTargetPos.getMagnitude());
                    telemetry.addData("locY:", camRelTargetPos.matrixMultiplied(cameraToTarget).y);

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

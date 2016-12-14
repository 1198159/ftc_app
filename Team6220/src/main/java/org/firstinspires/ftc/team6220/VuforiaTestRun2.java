package org.firstinspires.ftc.team6220;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * OpMode for testing out Vuforia. If you haven't learned how to use Vuforia, see the OpMode below
 *
 * TODO: Make the @see a functioning link
 * @see org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation
 */
@Autonomous(name = "Vuforia Test Run 2", group = "6220")
public class VuforiaTestRun2 extends LinearOpMode
{
    // Vuforia variables
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaTrackables visionTargets;
    VuforiaTrackable[] targets = new VuforiaTrackable[4];
    VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[4];

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    // Constants for later reference
    public static final String PICTURE_ASSET = "FTC_2016-17";
    public static final String VUFORIA_KEY = "ARnvYoH/////AAAAGQS+OAV8SElJhcYQ4Ud8LkNvhk/zpT8UiiVlkQOGgruNCfQryIqNOyyl6iYhvsCCVYMqHZPJJgORL7ZL3+Hl1VE/CJZiBI357gU4uSmFahasqA9UV/HVmd0Mze0j5cEaVgJ7w3dRhz4Lvdk7qcVwGQTMpAVFKdlpt0657wA0C2vFWzJgZZv3vk7Ouw6bfSltX1/Wgf15jcCcBPRLQs/KkIngbvc+rtBxtD5f4REyb9FuqtN00MoHKL8RIpFQagX/b39JbN8oFLDjUiC5smxchqIHYMIvt7JAQH0TT+fizeIYMnZk3/t8SfNg/gt1lJACY514k9TpM4UwfBvVZcfDVdXj1wKUsPWw8ndUQ6l5PtSq";

    public static final int WHEELS = 0;
    public static final int TOOLS = 1;
    public static final int LEGOS = 2;
    public static final int GEARS = 3;

    public static final int RED_LEFT = 0;
    public static final int RED_RIGHT = 1;
    public static final int BLUE_LEFT = 2;
    public static final int BLUE_RIGHT = 3;

    public static final float MM_PER_INCH = 25.4f;
    public static final float MM_BOT_SIZE = 18 * MM_PER_INCH;
    public static final float MM_FIELD_SIZE = 12 * 12 * MM_PER_INCH;

    @Override
    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        waitForStart();

        // Start tracking targets
        visionTargets.activate();

        while (opModeIsActive())
        {
            lastKnownLocation = getLatestLocation();

            // Inform drivers of robot location. Location is null if we lose track of targets
            if(lastKnownLocation != null)
                telemetry.addData("Pos", format(lastKnownLocation));
            else
                telemetry.addData("Pos", "Unknown");

            telemetry.update();
            idle();
        }
    }

    public OpenGLMatrix getLatestLocation()
    {
        // Checks each target to see if we can find our location. If none are visible, then it returns null
        for(int i = 0; i < targets.length; i++)
        {
            // Try to find location from this target
            OpenGLMatrix latestLocation = listeners[i].getUpdatedRobotLocation();

            // We've found a target to track
            if(latestLocation != null)
            {
                // Tell driver which target is being tracked
                telemetry.addData("Tracking", targets[i].getName());
                return latestLocation;
            }
        }
        // We've lost track of the targets
        telemetry.addData("Tracking", "lost");
        return lastKnownLocation;
    }

    public void setupVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Initialize vision targets
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset(PICTURE_ASSET);

        targets[RED_LEFT] = visionTargets.get(GEARS);
        targets[RED_LEFT].setName("Target Red Left");

        targets[RED_RIGHT] = visionTargets.get(TOOLS);
        targets[RED_RIGHT].setName("Target Red Right");

        targets[BLUE_LEFT] = visionTargets.get(LEGOS);
        targets[BLUE_LEFT].setName("Target Blue Left");

        targets[BLUE_RIGHT] = visionTargets.get(WHEELS);
        targets[BLUE_RIGHT].setName("Target Blue Right");

        // Set vision target locations on field. Origin is at corner of field between driver
        // stations, with the positive x-axis extending to the blue side, and positive y-axis
        // extending to the red side
        targets[RED_LEFT].setLocation(setMatrixLocation(1524, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[RED_RIGHT].setLocation(setMatrixLocation(2743.2f, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[BLUE_LEFT].setLocation(setMatrixLocation(MM_FIELD_SIZE, 2743.2f, 0, 90, 0, -90));
        targets[BLUE_RIGHT].setLocation(setMatrixLocation(MM_FIELD_SIZE, 1524, 0, 90, 0, -90));

        // Set phone location on robot, and inform listeners
        phoneLocation = setMatrixLocation(0, MM_BOT_SIZE / 2, 0, 90, 0, 0);

        for(int i = 0; i < targets.length; i++)
        {
            listeners[i] = (VuforiaTrackableDefaultListener) targets[i].getListener();
            listeners[i].setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }
    }

    // Creates a matrix for defining locations of things. Coordinates are given by x, y, and z, and
    // rotations about axes are given by u, v, and w. Default rotation order is XYZ.
    public OpenGLMatrix setMatrixLocation(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats location to something readable
    String format(OpenGLMatrix transformationMatrix)
    {
        return transformationMatrix.formatAsTransform();
    }
}

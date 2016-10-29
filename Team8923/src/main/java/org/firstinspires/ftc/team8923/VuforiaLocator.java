package org.firstinspires.ftc.team8923;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

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

import java.util.Arrays;

/*
 * This class is used for determining the robot location on the field by using Vuforia
 */
class VuforiaLocator
{
    // Vuforia variables
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable[] targets = new VuforiaTrackable[4];
    private VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[4];

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    // Constants for later reference
    private static final String PICTURE_ASSET = "FTC_2016-17";
    private static final String VUFORIA_KEY = "ARnvYoH/////AAAAGQS+OAV8SElJhcYQ4Ud8LkNvhk/zpT8UiiVlkQOGgruNCfQryIqNOyyl6iYhvsCCVYMqHZPJJgORL7ZL3+Hl1VE/CJZiBI357gU4uSmFahasqA9UV/HVmd0Mze0j5cEaVgJ7w3dRhz4Lvdk7qcVwGQTMpAVFKdlpt0657wA0C2vFWzJgZZv3vk7Ouw6bfSltX1/Wgf15jcCcBPRLQs/KkIngbvc+rtBxtD5f4REyb9FuqtN00MoHKL8RIpFQagX/b39JbN8oFLDjUiC5smxchqIHYMIvt7JAQH0TT+fizeIYMnZk3/t8SfNg/gt1lJACY514k9TpM4UwfBvVZcfDVdXj1wKUsPWw8ndUQ6l5PtSq";

    private static final int WHEELS = 0;
    private static final int TOOLS = 1;
    private static final int LEGOS = 2;
    private static final int GEARS = 3;

    private static final int RED_LEFT = 0;
    private static final int RED_RIGHT = 1;
    private static final int BLUE_LEFT = 2;
    private static final int BLUE_RIGHT = 3;

    private static final float MM_PER_INCH = 25.4f;
    private static final float MM_BOT_SIZE = 18 * MM_PER_INCH;
    private static final float MM_FIELD_SIZE = 12 * 12 * MM_PER_INCH;

    VuforiaLocator()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Stuff for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaLocalizer.setFrameQueueCapacity(1);

        // Initialize vision targets
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset(PICTURE_ASSET);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

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
        // extending to the red side. Units in mm and degrees
        // TODO: Should we use constants for these?
        targets[RED_LEFT].setLocation(createMatrix(1524, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[RED_RIGHT].setLocation(createMatrix(2743.2f, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[BLUE_LEFT].setLocation(createMatrix(MM_FIELD_SIZE, 2743.2f, 0, 90, 0, -90));
        targets[BLUE_RIGHT].setLocation(createMatrix(MM_FIELD_SIZE, 1524, 0, 90, 0, -90));

        // Set phone location on robot. Center of the camera is the origin
        phoneLocation = createMatrix(0, 0, 0, 90, 0, -90);

        // Setup listeners
        for(int i = 0; i < targets.length; i++)
        {
            listeners[i] = (VuforiaTrackableDefaultListener) targets[i].getListener();
            listeners[i].setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

        // Make not null to avoid errors
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }

    // Used to find the color of a pixel from the camera. Parameters are actually a coordinate
    // relative to vision target origin. Will need to use Colors class to extract RGB values
    int getPixelColor(int x, int y, int z) throws InterruptedException
    {
        // Get the latest frame object from Vuforia
        VuforiaLocalizer.CloseableFrame frame = vuforiaLocalizer.getFrameQueue().take();
        Bitmap bm = null;

        // The frame object contains multiple images in different formats. We want to store the
        // RGB565 image in our bitmap object. Not sure what the other formats do
        for(int i = 0; i < frame.getNumImages(); i++)
        {
            // We only want the rgb image
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
            {
                // Store image in the bitmap
                Image image = frame.getImage(i);
                bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(image.getPixels());
                break;
            }
        }

        // Make sure we actually have something
        if(bm == null)
            return 0;

        // Check to see if any of the vision targets are being tracked
        for(int i = 0; i < targets.length; i++)
        {
            OpenGLMatrix targetPose = listeners[i].getRawPose();

            // If the vision target isn't being tracked, it will be null. If it's not null, we're tracking it
            if(targetPose != null)
            {
                Matrix34F matrixPose = new Matrix34F();
                matrixPose.setData(Arrays.copyOfRange(targetPose.transposed().getData(), 0, 12));

                // Get the pixel that represents the specified point
                Vec2F point = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), matrixPose, new Vec3F(x, y, z));

                // Return the color of that pixel
                try
                {
                    return bm.getPixel((int) point.getData()[0], (int) point.getData()[1]);
                }
                catch(IllegalArgumentException e)
                {
                    // Pixel location was outside of camera field of view
                    return 0;
                }
            }
        }
        // None of the vision targets are being tracked
        return 0;
    }

    void startTracking()
    {
        visionTargets.activate();
    }

    float getRobotAngle()
    {
        updateLocation();
        return Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }

    // Returns an array where index 0 is for x, 1 for y, and 2 for z.
    float[] getRobotLocation()
    {
        updateLocation();
        return lastKnownLocation.getTranslation().getData();
    }

    boolean isTracking()
    {
        for(int i = 0; i < targets.length; i++)
        {
            if(listeners[i].isVisible())
                return true;
        }
        return false;
    }

    private void updateLocation()
    {
        // Checks each target to see if we can find our location. If none are visible, then it returns null
        for(int i = 0; i < targets.length; i++)
        {
            // Try to find location from this target
            OpenGLMatrix latestLocation = listeners[i].getUpdatedRobotLocation();

            // We've found a target to track
            if(latestLocation != null)
            {
                lastKnownLocation = latestLocation;
                return;
            }
        }
        // Location is unknown, so don't change anything
    }

    // Creates a matrix for defining locations of things. Coordinates are given by x, y, and z, and
    // rotations about axes are given by u, v, and w. Default rotation order is XYZ.
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
}

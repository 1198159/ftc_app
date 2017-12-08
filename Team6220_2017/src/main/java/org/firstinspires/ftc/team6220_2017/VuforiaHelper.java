package org.firstinspires.ftc.team6220_2017;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
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
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;


/**
 * This class contains useful methods for integrating vuforia into robot code
 */

public class VuforiaHelper
{
    // Vuforia variables------------------------------------
    VuforiaLocalizer vuforiaLocalizer;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    // Enum that has 4 possible values: UNKNOWN, LEFT, CENTER, and RIGHT
    public RelicRecoveryVuMark vuMark;

    boolean isVuMarkVisible;
    boolean isTracking = false;
    //-------------------------------------------------------

    // Variables for jewel color determination---------------
    float[] colorTransfer = new float[]{0, 0, 0};
     // What we will eventually return
    float[] colorOutput = new float[]{0, 0, 0};
     // Sum of all colors from sample pixels
    float[] colorSum = new float[]{0, 0, 0};

     // Integer that will be used to store individual pixel colors each loop of getAverageJewelColor()
    int color = 0;

    float colorLeft = 0;
    float colorRight = 0;

    float avgLeftJewelColor = 0;
    float avgRightJewelColor = 0;

    BlueJewel blueJewel = BlueJewel.UNDETERMINED;

     // Will be used to tell us which jewel is blue
    enum BlueJewel
    {
        LEFT,
        RIGHT,
        UNDETERMINED
    }
    //-------------------------------------------------------

    // Necessary matrices
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    Matrix34F rawPose;

    Bitmap bitMap;

    OpenGLMatrix pose;

    // key needed to use vuforia
    public static final String VUFORIA_KEY = "ARnvYoH/////AAAAGQS+OAV8SElJhcYQ4Ud8LkNvhk/zpT8UiiVlkQOGgruNCfQryIqNOyyl6iYhvsCCVYMqHZPJJgORL7ZL3+Hl1VE/CJZiBI357gU4uSmFahasqA9UV/HVmd0Mze0j5cEaVgJ7w3dRhz4Lvdk7qcVwGQTMpAVFKdlpt0657wA0C2vFWzJgZZv3vk7Ouw6bfSltX1/Wgf15jcCcBPRLQs/KkIngbvc+rtBxtD5f4REyb9FuqtN00MoHKL8RIpFQagX/b39JbN8oFLDjUiC5smxchqIHYMIvt7JAQH0TT+fizeIYMnZk3/t8SfNg/gt1lJACY514k9TpM4UwfBvVZcfDVdXj1wKUsPWw8ndUQ6l5PtSq";

    //used for storing info from vuforia frame queue to be analyzed in other methods
    VuforiaLocalizer.CloseableFrame frame;

    /*
    Necessary image variables
    Note:  we are using RGB888 because it stores more pixels, so we suspect
    that we will obtain more accurate colors by using it.  However, we are not certain about this
    and will need to look further into the issue
    */
    // todo find where to initialize and use
    Image image = null;
    int imageFormat;

    public void setupVuforia()
    {
        // Set parameters for VuforiaLocalizer (an interface supporting localization through visual
        // means) and create it
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        // Make sure that vuforia doesn't begin racking up unnecessary frames
        vuforiaLocalizer.setFrameQueueCapacity(1);

        // Initialize vision targets; this is important!-----------------
        relicTrackables = this.vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
         // For debugging purposes
        relicTemplate.setName("relicVuMarkTemplate");
        //---------------------------------------------------------------

        /*
         Set phone location on robot. The center of the camera is the origin.
         This location is 90 degrees less than the phone's actual rotation about the z-axis since we
         use the mathematical sense of absolute rotation (0 degrees located on the x-axis) while vuforia
         uses compass coordinates for rotation (0 degrees located on the y-axis)
        */
        // todo Finish adjusting for this year's robot; check y-axis rot fix for phone camera being rotated 180 deg
        phoneLocation = createMatrix(220, 0, 0, -90, 0, 180);

        // Avoids nullpointer errors
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        // Begin tracking targets before match
        startTracking();
    }

    /*
    // function for finding the location of robot
    public OpenGLMatrix getLatestLocation()
    {
        // checks each target to see if we can find our location. If none are visible, then it returns null
        for(int i = 0; i < targets.length; i++)
        {
            // try to find location from this target
            OpenGLMatrix latestLocation = listeners[i].getUpdatedRobotLocation();

            // we've found a target to track
            if(latestLocation != null)
            {
                // tell driver which target is being tracked
                // telemetry.addData("Tracking", targets[i].getName());
                return latestLocation;
            }
        }
        // we've lost track of the targets
        // telemetry.addData("Tracking", "lost");
        return lastKnownLocation;
    }

    //todo change formatting from targets (an array) to relictrackables (an arrayList)
    public void updateLocation()
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

    float[] getRobotLocation()
    {
        updateLocation();
        return lastKnownLocation.getTranslation().getData();
    }
    */

    public RelicRecoveryVuMark getVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    //check to make sure we can see the vuMark
    public boolean isVisible()
    {
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            isVuMarkVisible = true;
        else
            isVuMarkVisible = false;

        return isVuMarkVisible;
    }

    //double distance = 0;
    public double positionOfGoal()
    {
        if(vuMark == RelicRecoveryVuMark.RIGHT)
        {
            //change the distance traveled by to the Right position
            //this will need to be a constant
            return 1.8;
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT)
        {
            //change the distance fo Left
            return 1.2;
        }
        else
        {
            //change the distance for the center
            return 1.5;
        }
    }

    void startTracking()
    {
        relicTrackables.activate();
        isTracking = true;
    }

    void stopTracking()
    {
        relicTrackables.deactivate();
        isTracking = false;
    }

    // Formats location to something readable
    String format(OpenGLMatrix transformationMatrix)
    {
        return transformationMatrix.formatAsTransform();
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // todo Add capability to throw out pixels that are bad (e.g., off jewels or inside jewel holes)
    // Returns the average hue determined from a sample pixel area
    public float getAverageJewelColor(int x, int y)
    {
        // Reset transfer, sum, and output arrays to ensure that we don't accidentally reuse old values
        colorTransfer[0] = 0;
        colorTransfer[1] = 0;
        colorTransfer[2] = 0;
        colorSum[0] = 0;
        colorSum[1] = 0;
        colorSum[2] = 0;
        colorOutput[0] = 0;
        colorOutput[1] = 0;
        colorOutput[2] = 0;
        //Arrays.fill(colorTransfer, 0);
        //Arrays.fill(colorOutput, 0);

        // Ensure that we do not attempt to take pixels from outside the image border
        if (x >= 0 && x < Constants.IMAGE_WIDTH - Constants.JEWEL_SAMPLE_LENGTH / 2 && y >= 0 &&
                y < Constants.IMAGE_HEIGHT - Constants.JEWEL_SAMPLE_LENGTH / 2)
        {
            for (int j = y - Constants.JEWEL_SAMPLE_LENGTH / 2; j < y + Constants.JEWEL_SAMPLE_LENGTH / 2; j++) // Columns
            {
                for (int i = x - Constants.JEWEL_SAMPLE_LENGTH / 2; i < x + Constants.JEWEL_SAMPLE_LENGTH / 2; i++) // Rows
                {
                    // Get color of pixel.  This is actually a single integer that stores
                    // ARGB values in 4 bytes
                    color = bitMap.getPixel(i, j);

                    // Convert color integer to HSV (hue, sat, val)
                    // Hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorTransfer);

                    // Sum the HSV values of all pixels in bitmap
                    colorSum[0] += colorTransfer[0];

                    // Draw white border around sample region for debugging
                    if ((j == y - Constants.JEWEL_SAMPLE_LENGTH / 2 + 1) || (j == y + Constants.JEWEL_SAMPLE_LENGTH / 2 - 1)
                            || (i == x - Constants.JEWEL_SAMPLE_LENGTH / 2 + 1) || (i == x + Constants.JEWEL_SAMPLE_LENGTH / 2 - 1))
                    {
                        bitMap.setPixel(i, j, 0xffffffff);
                    }
                }
            }
            // Normalize output for 32x32 = 4096 pixel square above
            colorOutput[0] = colorSum[0] / 4096;
        }
        return colorOutput[0]; // Return the average hue
    }

    // Uses a bitmap created by vuforia to determine the saturations of the jewels and compare them,
    // then returns information that tells you whether the left jewel is red or blue
    public BlueJewel getLeftJewelColor() throws InterruptedException
    {
        // Vuforia code that uses the vuMark to determine relative positions of objects
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose != null)
        {
            rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // todo Test jewel locations
            // Place points where we think the centers of the jewels are relative to the vuMark
            Vec2F jewelLeft = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPose, new Vec3F(150, -220, -102));
            Vec2F jewelRight = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPose, new Vec3F(390, -220, -102));

            // Get the latest frame from vuforia
            frame = vuforiaLocalizer.getFrameQueue().take();

            long numImages = frame.getNumImages();

            for (int j = 0; j < numImages; j++)
            {
                image = frame.getImage(j);
                imageFormat = image.getFormat();

                if (imageFormat == PIXEL_FORMAT.RGB565) break;
            }

            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();

            // Create bitmap of image to detect color
            bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bitMap.copyPixelsFromBuffer(image.getPixels());

            // Boundaries of image
            int lx = (int) jewelLeft.getData()[0];
            int ly = (int) jewelLeft.getData()[1];

            int rx = (int) jewelRight.getData()[0];
            int ry = (int) jewelRight.getData()[1];

            avgLeftJewelColor = getAverageJewelColor(lx, ly);
            avgRightJewelColor = getAverageJewelColor(rx, ry);

            // todo Make sure new logic for shifting red value is correct
            // Adjust color for red range.  Red color on the jewel can be on either side of 0, so
            // we must subtract 360 if red values are left of 0 to ensure blue is greater than red
            colorLeft = (avgLeftJewelColor > 315) ? avgLeftJewelColor - 360 : avgLeftJewelColor;
            colorRight = (avgRightJewelColor > 315) ? avgRightJewelColor - 360 : avgRightJewelColor;
        }

        //todo Adjust value
        // Check to see if our color samples were actually on the jewel
        // If value tested is negative, then left jewel is blue.  Otherwise, right is blue
        if (Math.abs(colorLeft - colorRight) > 5)
        {
            if ((colorLeft - colorRight) > 0)
                blueJewel = BlueJewel.LEFT;
            else
                blueJewel = BlueJewel.RIGHT;
        }

        return blueJewel;
    }
}

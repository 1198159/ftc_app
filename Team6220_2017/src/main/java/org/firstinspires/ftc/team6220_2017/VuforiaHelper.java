package org.firstinspires.ftc.team6220_2017;

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

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;


import java.util.Arrays;


/**
 * Created by Colew on 10/25/2016.
 */

public class VuforiaHelper
{
    // Vuforia variables
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaTrackables visionTargets;
    VuforiaTrackable[] targets = new VuforiaTrackable[4];
    VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[4];

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    Bitmap bitMap;

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

    //CodeReview: The botsize is a constant that relate to your robot - should it be in MasterOpMode? (Probably) (Well, definitely)
    //            Except that it's not actually used... hmm. So maybe delete it :)
    public static final float MM_PER_INCH = 25.4f;
    public static final float MM_BOT_SIZE = 18 * MM_PER_INCH;
    public static final float MM_FIELD_SIZE = 12 * 12 * MM_PER_INCH;

    //stores colors used as return values in getJewelColor()
    enum JewelColor
    {
        red,
        blue,
        undetermined
    }
    //function for finding the location of robot
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
                //telemetry.addData("Tracking", targets[i].getName());
                return latestLocation;
            }
        }
        // We've lost track of the targets
        //telemetry.addData("Tracking", "lost");
        return lastKnownLocation;
    }


    //finds color of image
    public float[] getImageColor(Image image, Vec2F targetUpperLeftCorner)
    {
        //what we want to return
        float[] colorOutput = new float[]{0,0,0};

        if (image != null)
        {
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            //coordinates of corner of target area for acquiring color
            int x = (int) targetUpperLeftCorner.getData()[0];
            int y = (int) targetUpperLeftCorner.getData()[1];
            //color of sample pixel from nested for loop below
            int samplePixel = 0;

            //sum of all colors from sample pixels
            float[] colorSum = new float[]{0,0,0};

            float[] colorHSV = new float[]{0,0,0};

            // create image bitmap to get color
            bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bitMap.copyPixelsFromBuffer(image.getPixels());

            for (int yComp = y; yComp < y + 300; y += 30)
            {
                for (int xComp = x; xComp < x + 650; x += 65)
                {
                    samplePixel = bitMap.getPixel(xComp, yComp);

                    Color.colorToHSV(samplePixel, colorHSV);

                    // sum HSV colors of all sample pixels
                    for (int i = 0; i < 3; i++)
                    {
                        colorSum[i] += colorHSV[i];
                    }


                    //add black border around pixel area for debugging.  Sets each pixel on border to black.
                    if ((yComp == y) || (yComp == y + 300) || (xComp == x) || (xComp == x +650))
                    {
                        bitMap.setPixel(xComp, yComp, 0);
                    }

                }
            }

            //averages colors of sample pixels
            for (int i = 0; i < 3; i++)
            {
                colorOutput[i] = colorSum[i] /121;
            }
        }

        return colorOutput;
    }

    public JewelColor getJewelColor()
    {
        //Image image ;
        //Vec2F targetUpperLeftCorner;
        JewelColor jewelColor = JewelColor.undetermined;

        //float[] colorOutput = getImageColor(image, targetUpperLeftCorner);
        boolean isBlue = false;
        boolean isRed = false;


        if(isBlue)
        {

        }
        return jewelColor;
    }

    public void setupVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaLocalizer.setFrameQueueCapacity(1);

        // Initialize vision targets

        /*
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
        // TODO: use constants
        targets[RED_LEFT].setLocation(createMatrix(1524, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[RED_RIGHT].setLocation(createMatrix(2743.2f, MM_FIELD_SIZE, 0, 90, 0, 0));
        targets[BLUE_LEFT].setLocation(createMatrix(MM_FIELD_SIZE, 2743.2f, 0, 90, 0, -90));
        targets[BLUE_RIGHT].setLocation(createMatrix(MM_FIELD_SIZE, 1524, 0, 90, 0, -90));
        */

        //Sets phone location on robot. The center of the camera is the origin
        //This location is 90 degrees less than the phone's actual rotation about the z-axis since we
        //use the mathematical sense of absolute rotation (0 degrees located on the x-axis) while vuforia
        //uses compass coordinates for rotation (0 degrees located on the y-axis)
        phoneLocation = createMatrix(-70, 220, 0, 90, 0, 180);
        //phoneLocation = createMatrix(-70, 220, 0, 90, 0, 180);

        // Setup listeners
        for(int i = 0; i < targets.length; i++)
        {
            listeners[i] = (VuforiaTrackableDefaultListener) targets[i].getListener();
            listeners[i].setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

        // avoids nullpointer errors
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }

    // Used to find pixel color from the camera. Parameters are actually a coordinate
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
        {
            return 0;
        }
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

    void startTracking()
    {
        visionTargets.activate();
    }

    void stopTracking() {visionTargets.deactivate();}

    boolean isTracking()
    {
        for(int i = 0; i < targets.length; i++)
        {
            if(listeners[i].isVisible())
            {
                return true;
            }
        }
        return false;
    }

    float[] getRobotLocation()
    {
        updateLocation();
        return lastKnownLocation.getTranslation().getData();
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
}

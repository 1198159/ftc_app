package org.firstinspires.ftc.team6220_2017;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

import java.nio.ByteBuffer;
import java.util.Arrays;


/**
 * This class contains useful methods for integrating vuforia into robot code
 */

public class VuforiaHelper
{
    // Vuforia variables
    VuforiaLocalizer vuforiaLocalizer;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    //enum which has 4 possible values: UNKNOWN, LEFT, CENTER, and RIGHT
    public RelicRecoveryVuMark vuMark;
    //VuforiaTrackable[] targets = new VuforiaTrackable[1];
    //VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[1];
    //

    // colors of both jewels used in getJewelColor()
    public float leftJewelColorHSV[] = {0f, 0f, 0f};
    public float rightJewelColorHSV[] = {0f, 0f, 0f};

    float[] colorTransfer = new float[]{0,0,0};
    //what we will eventually return
    float[] colorOutput = new float[]{0,0,0};
    //sum of all colors from sample pixels
    float[] colorSum = new float[]{0, 0, 0};

    int color = 0;
    //intermediary jewel color arrays that will be used by getImage()
    float leftColorOutput[] = {0, 0, 0};
    float rightColorOutput[] = {0, 0, 0};

    float avgLeftJewelColor = 0;
    float avgRightJewelColor = 0;

    float colorLeft;
    float colorRight;

    boolean isLeftJewelBlue;
    boolean isVuMarkVisible;

    // necessary matrices
    OpenGLMatrix targetPosition;
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

    // stores colors used as return values in getJewelColor()
    enum JewelColor
    {
        red,
        blue,
        undetermined
    }

    public void setupVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        this.vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        //setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        //make sure that vuforia doesn't begin racking up unnecessary frames
        vuforiaLocalizer.setFrameQueueCapacity(1);

        //---------------------------
        // Initialize vision targets; this is important!
        relicTrackables = this.vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        // for debugging, but not essential
        relicTemplate.setName("relicVuMarkTemplate");
        //----------------------------

        /*
         Set phone location on robot. The center of the camera is the origin.
         This location is 90 degrees less than the phone's actual rotation about the z-axis since we
         use the mathematical sense of absolute rotation (0 degrees located on the x-axis) while vuforia
         uses compass coordinates for rotation (0 degrees located on the y-axis)
        */
        //todo adjust for this year's robot
        phoneLocation = createMatrix(220, 0, 0, 90, 0, 180);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /*
        // Setup listeners
        for(int i = 0; i < targets.length; i++)
        {
            listeners[i] = (VuforiaTrackableDefaultListener) targets[i].getListener();
            listeners[i].setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }
        */

        // avoids nullpointer errors
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        //start tracking targets
        relicTrackables.activate();
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

    //todo change formatting from targets (an array) to relictrackables (a list)
    /*
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
            targetPosition = listeners[i].getRawPose();

            // If the vision target isn't being tracked, it will be null. If it's not null, we're tracking it
            if(targetPosition != null)
            {
                Matrix34F matrixPose = new Matrix34F();
                matrixPose.setData(Arrays.copyOfRange(targetPosition.transposed().getData(), 0, 12));

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

//    void startTracking()
//    {
//        visionTargets.activate();
//    }
//
//    void stopTracking() {visionTargets.deactivate();}

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
    */

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

    //encapsulates function necessary to average color values of jewels
    public float getAverageJewelColor(int x, int y)
    {
        if (x >= 0 && x < 1280 - 32 && y >= 0 && y < 720 - 32)
        {
            for (int j = y - 32; j < y + 32; j++) // columns
            {
                for (int i = x - 32; i < x + 32; i++) // rows
                {
                    // get RGB color of pixel
                    color = bitMap.getPixel(i, j);

                    // convert RGB to HSV - hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorTransfer);

                    // integrate HSV color of all pixels
                    colorSum[0] += colorTransfer[0];

                    // draw black border around sample region for debugging only
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31))
                    {
                        bitMap.setPixel(i, j, 0xff00ff00);
                    }
                }
            }
            // normalize output for 32x32 = 4096 integration above
            colorOutput[0] = colorSum[0] / 4096;
        }
        return colorOutput[0]; // return the averaged sampled HSV color value
    }

    /*
    Note:  this method references the color of the left jewel for its output

    uses getImageColor() to determine the colors of the jewels and compare them, then returns
    information that tells you whether the left jewel is red or blue
    */
    public boolean getLeftJewelColor() throws InterruptedException
    {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose != null)
        {
            rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            Vec2F jewelLeft = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPose, new Vec3F(165, -175, -102));
            Vec2F jewelRight = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPose, new Vec3F(390, -180, -102));

            // takes the frame at the head of the queue
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

            // create bitmap of image to detect color
            bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bitMap.copyPixelsFromBuffer(image.getPixels());

            // coordinates in image
            // TODO: check to make sure x < 1280; y < 720
            int lx = (int) jewelLeft.getData()[0];
            int ly = (int) jewelLeft.getData()[1];

            int rx = (int) jewelRight.getData()[0];
            int ry = (int) jewelRight.getData()[1];

            avgLeftJewelColor = getAverageJewelColor(lx, ly); // get the averaged jewel HSV color value for the left jewel
            avgRightJewelColor = getAverageJewelColor(rx, ry);

            // adjust color for red range (if red is between 0 and 45 degrees, shift by adding 300 so that red is greater than blue
            colorLeft = (avgLeftJewelColor < 45) ? avgLeftJewelColor + 360 : avgLeftJewelColor;
            colorRight = (avgRightJewelColor < 45) ? avgRightJewelColor + 360 : avgRightJewelColor;
        }
        //deltaColorHSV = colorLeft - colorRight;
        // if left color is negative, then left side is blue
        if ((colorLeft - colorRight) < 0) isLeftJewelBlue = true; // BLUE
        else isLeftJewelBlue = false; // RED

        return isLeftJewelBlue;
    }

    //finds color of image
    public float[] getImageColor(Image image, Vec2F targetUpperLeftCorner)
    {
        if (image != null)
        {
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            //coordinates of corner of target area for acquiring color
            int x = (int) targetUpperLeftCorner.getData()[0];
            int y = (int) targetUpperLeftCorner.getData()[1];

            //color of sample pixel from nested for loop below; will later be given an HSV value
            int samplePixel = 0;

            //sum of all colors from sample pixels
            float[] colorSum = new float[]{0, 0, 0};

            //intermediary array necessary for calculations below
            float[] colorHSV = new float[]{0, 0, 0};

            // create image bitmap to get color
            //bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.ARGB_8888);
            //bitMap.copyPixelsFromBuffer(image.getPixels());
            //ByteBuffer buffer = image.getPixels();
            //bitMap.copyPixelsFromBuffer(buffer);

            samplePixel = image.getPixels().get(0);

            //todo figure out why values can be negative and all read identical values in telemetry
            //samplePixel = bitMap.getPixel(0, 0);
            int R = (samplePixel & 0xff0000) >> 16;
            int G = (samplePixel & 0x00ff00) >> 8;
            int B = (samplePixel & 0x0000ff) >> 0;
            colorOutput[0] = image.getPixels().get(0);
            colorOutput[1] = image.getPixels().get(1);
            colorOutput[2] = image.getPixels().get(2);
        }
        return colorOutput;


/*
            //todo adjust pixel numbers when ready; maybe y = 30 and x = 65, since vuforia uses 1/10 pixels of jpeg
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
                    if ((yComp == y) || (yComp == y + 300) || (xComp == x) || (xComp == x + 650))
                    {
                        bitMap.setPixel(xComp, yComp, 0);
                    }
                }
            }

            //average colors of sample pixels by dividing by # of samples
            for (int i = 0; i < 3; i++)
            {
                colorOutput[i] = colorSum[i] / 121;
            }
        }
        */

            //return colorOutput;
    }


    /*
    Note:  this method references the color of left jewel for its output

    uses getImageColor() to determine the colors of the jewels and compare them, then returns
    information that tells you whether the left jewel is red or blue
    */
    public JewelColor getJewelColor(OpMode opMode) throws InterruptedException
    {
        //initialize jewel color value to prevent possibility of a null pointer error
        JewelColor leftJewelColor = JewelColor.undetermined;

        //once again sets all array outputs to 0 to prevent holdover values from previous method calls
        leftColorOutput[0] = 0;
        leftColorOutput[1] = 0;
        leftColorOutput[2] = 0;
        rightColorOutput[0] = 0;
        rightColorOutput[1] = 0;
        rightColorOutput[2] = 0;

        //todo uncomment and finish
        //for (int i = 0; i < listeners.length; i++)
        {
            //check to make sure that we are tracking the image
            //if (listeners[i].isVisible())
            {
                //frame that will be analyzed here
                frame = vuforiaLocalizer.getFrameQueue().take();

                long numberOfImages = frame.getNumImages();

                //use j, since i is already used
                /*for (int j = 0; j < numberOfImages; j++)
                {
                    image = frame.getImage(j);
                    imageFormat = image.getFormat();

                    if (imageFormat == PIXEL_FORMAT.RGB888) break;
                }*/

                image = frame.getImage(0);
                imageFormat = image.getFormat();
                opMode.telemetry.addData("image width", image.getWidth());
                opMode.telemetry.addData("image height", image.getHeight());
                opMode.telemetry.update();

                //targetPosition = listeners[i].getRawPose();
                //if (targetPosition != null)
                {
                    /*
                    //math to get the position of of the current target point for vuforia
                    Matrix34F rawPosition = new Matrix34F();
                    float[] positionData = Arrays.copyOfRange(targetPosition.transposed().getData(), 0, 12);
                    rawPosition.setData(positionData);

                    //these are the coordinates for projected points in the upper left corner of each
                    //of the jewels; our image size is 650mm x 300mm
                    //todo check locations of projected points
                    Vec2F cornerJewelLeft = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPosition, new Vec3F(500, -800, 0));
                    Vec2F cornerJewelRight = Tool.projectPoint(vuforiaLocalizer.getCameraCalibration(), rawPosition, new Vec3F(2000, -800, 0));
                    */

                    Vec2F cornerJewelLeft = new Vec2F(10, 10);
                    Vec2F cornerJewelRight = new Vec2F(20, 20);

                    leftColorOutput = getImageColor(image, cornerJewelLeft);
                    rightColorOutput = getImageColor(image, cornerJewelRight);

                    // close frame to free up memory space
                   // frame.close();

                    /*
                    adjust color for red range (if red is between 0 and 90 degrees, shift by
                    adding 360 so that red is greater than blue)
                    Note:  logic is shorthand for if statement
                    */
                    float colorLeft = (leftColorOutput[0] < 90) ? leftColorOutput[0] + 360 : leftColorOutput[0];
                    float colorRight = (rightColorOutput[0] < 90) ? rightColorOutput[0] + 360 : rightColorOutput[0];

                    float deltaColorHSV = colorLeft - colorRight;
                    // if left color is negative, then left side is blue
                    if (deltaColorHSV < 0)
                    {
                        leftJewelColor = JewelColor.blue;
                    }
                    else
                    {
                        leftJewelColor = JewelColor.red;
                    }
                    // debug data
                    //telemetry.log().add(String.format("LeftSideHue: %f RightSideHue: %f", leftColorOutput[0], rightColorOutput[0]));
                }
            }
        }
        return leftJewelColor;
    }
}

package org.firstinspires.ftc.team417_2017;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.lang.reflect.Array;
import java.util.Arrays;

public class VuforiaDetection
{
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    int imageFormat;
    Bitmap bm; // android.graphics

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    public RelicRecoveryVuMark vuMark;
    Matrix34F rawPose;

    // old color variables for left and right jewels
    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};

    int color = 0;
    int colorRGB = 0;
    int avgColorRGB = 0;
    int sumColorRGB = 0;
    int colorR = 0;
    int colorG = 0;
    int colorB = 0;
    int avgColorR = 0;
    int avgColorG = 0;
    int avgColorB = 0;
    int sumColorR = 0;
    int sumColorG = 0;
    int sumColorB = 0;
    float[] colorHsvSum = {0,0,0};
    float[] colorHSV = {0,0,0};
    float[] colorHsvOut = {0,0,0};
    float avgLeftJewelColor;
    float avgRightJewelColor;

    boolean isVuMarkVisible;
    boolean isLeftJewelBlue;

    float leftRed;
    float leftBlue;
    float leftOther;
    float rightRed;
    float rightBlue;
    float rightOther;

    float colorLeft;
    float colorRight;
    float deltaColorHSV;

    int redPixels = 0;
    int bluePixels = 0;
    int otherPixels = 0;

    Vec2F jewelLeft;
    Vec2F jewelRight;

    OpenGLMatrix pose;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    WebcamName webcamName;

    public float GetAvgJewelColor(int x, int y)
    {
        if (x>=0 && x<1280-32 && y>=0 && y<720-32)
        {
            colorHsvSum[0] = 0;
            for (int j = y - 32; j < y + 32; j++) // columns
            {
                for (int i = x - 32; i < x + 32; i++) // rows
                {
                    // get RGB color of pixel
                    color = bm.getPixel(i, j);

                    // convert RGB to HSV - hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorHSV);

                    // integrate HSV color of all pixels
                    colorHsvSum[0] += colorHSV[0];

                    // draw black border around sample region for debugging only
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31))
                    {
                        bm.setPixel(i, j, 0xff00ff00);
                    }
                }
            }
            // normalize output for 32x32 = 4096 integration above
            colorHsvOut[0] = colorHsvSum[0] / 4096;
        }
        return colorHsvOut[0]; // return the averaged sampled HSV color value
    }

    /*
    This method returns the average RBG of the entire sampling area of one Jewel, then converts the
    averaged RGB value into an HSV value.
     */
    public float GetAvgRGBColor(int x, int y)
    {
        sumColorR = 0;
        sumColorG = 0;
        sumColorB = 0;

        if (x>=0 && x<1280-32 && y>=0 && y<720-32)
        {
            colorHsvSum[0] = 0;
            for (int j = y - 32; j < y + 32; j++) // columns
            {
                for (int i = x - 32; i < x + 32; i++) // rows
                {
                    // get RGB color of pixel
                    colorRGB = bm.getPixel(i, j);
                    colorR = (colorRGB>>16) & 0x000000FF;
                    colorG = (colorRGB>>8) & 0x000000FF;
                    colorB = colorRGB & 0x000000FF;

                    sumColorR += colorR;
                    sumColorG += colorG;
                    sumColorB += colorB;

                    // draw black border around sample region for debugging only
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31))
                    {
                        bm.setPixel(i, j, 0xff00ff00);
                    }
                }
            }
            // normalize output for 32x32 = 4096 integration above
            avgColorR = sumColorR / 4096;
            avgColorR = (avgColorR & 0x000000FF) << 16;
            avgColorG = sumColorG / 4096;
            avgColorG = (avgColorG & 0x000000FF) << 8;
            avgColorB = sumColorB / 4096;
            avgColorB = avgColorB & 0x000000FF;
            avgColorRGB = avgColorR + avgColorG + avgColorB;
        }
        // convert RGB to HSV - hue, sat, val
        // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
        Color.colorToHSV(avgColorRGB, colorHsvOut);

        return colorHsvOut[0]; // return the averaged sampled HSV color value
    }

    /* sortPixels */

    public float sortPixels(int x, int y)
    {
        redPixels = 0;
        bluePixels = 0;
        otherPixels = 0;

        if (x>=0 && x<1280-32 && y>=0 && y<720-32)
        {
            for (int j = y - 32; j < y + 32; j++) // columns
            {
                for (int i = x - 32; i < x + 32; i++) // rows
                {
                    // get RGB color of pixel
                    color = bm.getPixel(i, j);

                    // convert RGB to HSV - hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorHSV);

                    // checks the pixels
                    float hue = colorHSV[0];

                    if(hue >= 330 && hue <= 30) // range of red pixels
                    {
                        redPixels++; // identify pixel as red
                    }
                    else if(hue >= 210 && hue <= 270) // range of blue pixels
                    {
                        bluePixels++; // identify pixel as blue
                    }
                    else // if the pixel is not in the red or blue ranges defined above, identify it as other
                    {
                        otherPixels++;
                    }

                    // draw black border around sample region for debugging only
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31))
                    {
                        bm.setPixel(i, j, 0xff00ff00);
                    }
                }
            }
        }
        return colorHsvOut[0]; // return the averaged sampled HSV color value
    }

    /*
    This method returns whether the left jewel is blue or not and the vumark (left, center, or right).
     */
    public boolean GetLeftJewelColor() throws InterruptedException
    {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose!=null)
        {
            rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            jewelLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(150, -160, -102));
            jewelRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375, -160, -102));
        }
        else
        {
            // image size is 254 mm x 184 mm
            jewelLeft = new Vec2F(641, 503); // left jewel
            jewelRight = new Vec2F(1056, 494); // right jewel
        }
        // takes the frame at the head of the queue
        frame = vuforia.getFrameQueue().take();

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
        bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(image.getPixels());

        // coordinates in image
        // TODO: check to make sure x < 1280; y < 720
        int lx = (int) jewelLeft.getData()[0];
        int ly = (int) jewelLeft.getData()[1];

        int rx = (int) jewelRight.getData()[0];
        int ry = (int) jewelRight.getData()[1];

        //avgLeftJewelColor = GetAvgJewelColor(lx, ly); // get the averaged jewel HSV color value for the left jewel
        //avgRightJewelColor = GetAvgJewelColor(rx, ry);
        avgLeftJewelColor = GetAvgRGBColor(lx, ly);
        avgRightJewelColor = GetAvgRGBColor(rx, ry);

        // adjust color for red range (if red is between 0 and 45 degrees, shift by adding 300 so that red is greater than blue
        colorLeft = (avgLeftJewelColor < 45) ? avgLeftJewelColor + 300 : avgLeftJewelColor;
        colorRight = (avgRightJewelColor < 45) ? avgRightJewelColor + 300 : avgRightJewelColor;
        deltaColorHSV = colorLeft - colorRight;
        // if left color is negative, then left side is blue
        if (deltaColorHSV < 0) isLeftJewelBlue = true; // BLUE
        else isLeftJewelBlue = false; // RED
        return isLeftJewelBlue;
    }

    /* arrangePixels */

    public void arrangePixels() throws InterruptedException
    {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose!=null)
        {
            rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            Vec2F jewelLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(150, -160, -102));
            Vec2F jewelRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375, -160, -102));

            // takes the frame at the head of the queue
            frame = vuforia.getFrameQueue().take();

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
            bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            // coordinates in image
            // TODO: check to make sure x < 1280; y < 720
            int lx = (int) jewelLeft.getData()[0];
            int ly = (int) jewelLeft.getData()[1];

            int rx = (int) jewelRight.getData()[0];
            int ry = (int) jewelRight.getData()[1];

            // calculate number of red, blue and other pixels for the LEFT JEWEL
            sortPixels(lx, ly);
            leftBlue = bluePixels;
            leftRed = bluePixels;
            leftOther = bluePixels;
            sortPixels(rx, ry);
            rightBlue = redPixels;
            rightRed = redPixels;
            rightOther = redPixels;
        }
    }

    public RelicRecoveryVuMark GetVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public void initVuforia()
    {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //This licence key belongs to Bob Atkinson
        parameters.vuforiaLicenseKey = "AXcvdAD/////AAADmeAgjY5Fe0yHvh72y9/lFm8S1V6le66U/3YycNiUtC7rJicpxMsf7kzvkk8HOJj6AATgLTLyDIdTcPy/l7fGRAEjmjAqOXzNO4pi4BmTuXRLH3iLFY5w6hby2W9sh6R9HxWtA9Y6zKRTC3aVkWqUs6VBChVoMX7eweMT8YL12S+hKFndrKlQAsqeM66oXJ2MBXNBIt8UXwK+3We6YAKWktsvKo5x6d2X9C7qrgUl83vDHh7jqJUf0/gi9H77mavyT4Ds8cAv6K52SBmZjExOD6cxbYr4nAhreS/kgQHIPPJssUDqj5imYeQeDXRCeHLl5sz7+5+4csLJixo4irUhe27YvRDSfshvcjz0jIsne+YL";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set phone location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0.0f, 0.0f, 0.0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, -90, 0));  // landscape back camera

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1); // this tells VuforiaLocalizer to only store one frame at a time
        // wait until the start button is pressed

        relicTrackables.activate(); // activate tracking
    }


    public boolean isVisible()
    {
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) isVuMarkVisible = true;
        else isVuMarkVisible = false;
        return isVuMarkVisible;
    }


}


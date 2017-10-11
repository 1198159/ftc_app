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

import java.util.Arrays;

public class VuforiaNavigate
{
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    Image imageRGB888 = null;
    int imageFormat;
    Bitmap bm; // android.graphics
    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};


    VuforiaTrackables imageTargets;
    VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[0];

    OpenGLMatrix pose;

    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void initVuforia()
    {
        /*
        To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        If no camera monitor is desired, use the parameterless constructor instead.
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(); // took out the image displayed on the phone

        //This license key belongs to Steve Geffner
        parameters.vuforiaLicenseKey = "ATJf0AL/////AAAAGQZ9xp9L+k5UkmHj3LjxcoQwNTTBJqjO9LYsbkWQArRpYKQmt7vqe680RCQSS9HatStn1XZVi7rgA8T7qrJz/KYI748M4ZjlKv4Z11gryemJCRA9+WWkQ51D3TuYJbQC46+LDeMfbvcJQoQ79jtXr7xdFhfJl1mRxf+wMVoPWfN6Dhr8q3XVxFwOE/pM3gXWQ0kacbcGR/vy3NAsbOhf02DEe5WoV5PNZTF34LWN3dWURu7NJsnbFzkpzXdogeVAdiQ3QUWDvuhEwvSJY4W+fCTb15t6T/c/GJ/vqptsVKqavXk6MQobnUsVFpFP+5OSuRQe7EgvWuOxn7xn5YlC+CWAYh9LrXDpktwCwBAiX3Gx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // we are using the back camera
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1); // this tells VuforiaLocalizer to only store one frame at a time

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // set phone location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(-19.05f, 203.0f, 3.0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));  // portrait phone
    }

    void startTracking()
    {
        imageTargets.activate();
    }

    void stopTracking()
    {
        imageTargets.deactivate();
    }

    public boolean isVisible(int targetIndex)
    {
        return listeners[targetIndex].isVisible();
    }


    public enum JewelColor
    {
        red,
        blue,
        undefined
    }

    /*
    This method returns the left jewel color by sampling both of the jewels color and comparing both
    of them to see which one is redder and which is bluer.  WE get the image from Vuforia, and then
    project a point onto the image.
     */
    public JewelColor GetJewelColor() throws InterruptedException
    {
        JewelColor jColor = JewelColor.undefined; // initialize jewel color to be undefined
        for (int i = 0; i < listeners.length; i++)
        {
            if (listeners[i].isVisible()) // if tracking image
            {
                // takes the frame at the head of the queue
                frame = vuforia.getFrameQueue().take();
                image = null;
                imageRGB565 = null;
                imageRGB888 = null;

                long numImages = frame.getNumImages();

                for (int j = 0; j < numImages; j++)
                {
                    image = frame.getImage(j);
                    imageFormat = image.getFormat();

                    if (imageFormat == PIXEL_FORMAT.RGB565) break;
                }

                pose = listeners[i].getRawPose();
                if (pose != null)
                {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);
                    // image size is 254 mm x 184 mm
                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
// These are the coordinates for projecting points for each of the jewels
                    Vec2F jewelLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(140, -108, 0));
                    Vec2F jewelRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(292, -108, 0));

                    GetImageColor(image, jewelLeft, leftColorHSV);
                    GetImageColor(image, jewelRight, rightColorHSV);
                    frame.close(); // close frame to free memory

                    // adjust color for red range (if red is between 0 and 45 degrees, shift by adding 300 so that red is greater than blue
                    float colorLeft = (leftColorHSV[0] < 45) ? leftColorHSV[0] + 300 : leftColorHSV[0];
                    float colorRight = (rightColorHSV[0] < 45) ? rightColorHSV[0] + 300 : rightColorHSV[0];

                    float deltaColorHSV = colorLeft - colorRight;
                    // if left color is negative, then left side is blue
                    if (deltaColorHSV < 0)
                    {
                        jColor = JewelColor.blue; // BLUE
                    }
                    else
                    {
                        jColor = JewelColor.red; // RED
                    }
                    //telemetry.log().add(String.format("LeftSideHue: %f RightSideHue: %f", leftColorHSV[0], rightColorHSV[0]));
                }
            }
        }
        return jColor;
    }


    /*
    This method samples the area in an image to determine the color hue for each jewel, allowing us
    to detect red or blue.
     */
    public void GetImageColor(Image image, Vec2F jewelPoint, float[] colorHsvOut) throws InterruptedException
    {
        if (image != null)
        {
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            int stride = image.getStride();
            int color = 0;
            float[] colorHsvSum = {0,0,0};
            float[] colorHSV = {0,0,0};
            // coordinates in image
            int x = (int) jewelPoint.getData()[0];
            int y = (int) jewelPoint.getData()[1];

            // clip the x and y values to width and height
            x = (x < imageWidth - 8) ? x : imageWidth;
            y = (y < imageHeight - 8) ? y : imageHeight;
            // clip the x and y values >= 8
            x = (x >= 8) ? x : 8;
            y = (y >= 8) ? y : 8;

            // create bitmap of image to detect color
            bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            //telemetry.log().add(String.format("Width: %d, Height: %d, Stride: %d", imageWidth, imageHeight, stride));

            // sample and integrate colors for a 16x16 square region of pixels
            for (int j = y - 8; j < y + 8; j++) {
                for (int i = x - 8; i < x + 8; i++)
                {
                    // get RGB color of pixel
                    color = bm.getPixel(i, j);

                    // convert RGB to HSV - hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorHSV);

                    // integrate HSV color of all pixels
                    for (int k = 0; k < 3; k++)
                        colorHsvSum[k] += colorHSV[k];

                    // draw black border around sample region for debugging only
                    if ((j == y - 8) || (j == y + 7) || (i == x - 8) || (i == x + 7))
                    {
                        bm.setPixel(i, j, 0);
                    }
                }
            }
            // normalize output for 16x16 = 256 integration above
            for (int k = 0; k < 3; k++)
            {
                colorHsvOut[k] = colorHsvSum[k] / 256;
            }
            //telemetry.log().add(String.format("Hue: %f, Sat: %f, Val: %f", colorHSV[0], colorHSV[1], colorHSV[2]));
        }
    }
}


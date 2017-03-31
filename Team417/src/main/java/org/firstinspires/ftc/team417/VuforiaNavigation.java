package org.firstinspires.ftc.team417;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.util.RobotLog;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by user on 11/8/2016.
 */

public class VuforiaNavigation {
    OpenGLMatrix lastLocation = null; //CodeReview: it seems some code in other classes isn't handling this being null.
                                      //            Perhaps you should initialize it to something other than null.
                                      //            Also, I would recommend creating a method on this object that returns this value,
                                      //            instead of letting other classes read this variable directly.
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    Image imageRGB888 = null;
    int imageFormat;
    Bitmap bm;      // android.graphics
    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};


    VuforiaTrackables imageTargets;
    VuforiaTrackable[] trackableTargets = new VuforiaTrackable[4];
    VuforiaTrackableDefaultListener[] listeners = new VuforiaTrackableDefaultListener[4];

    OpenGLMatrix pose;

    //CodeReview: these constants are also declared in MasterAutonomous. You should declare them once (in MasterOpMode)
    //            so there's no chance that the values will be different in different files.
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

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(); // took out the image displayed on the phone
        parameters.vuforiaLicenseKey = "ARvv/bz/////AAAAGVPnoD3yyERZqCnr7KHm3FgrFIn4fQGOfufF8iBOidgC9FrsCDLKKjDvkgVJybQpEu3brET0BeFbyb746Cu/gT8E6S89bGDF2h6iSlKk4IRoB60EPX/p+yTPMr82hZRxS5SGDlU1JVarJwhN3el/donXpG70LYWRrInA/51wa+pSkM64ZdhRcVwTutBBoXvFMdil6YBwmHTgPK8TpGLHBFYlJskNT2FxdGQiB5JyXkzNLoNSE3MeUDBsdZ0H2X39BOqcEh0hgZrLQ33WuQBwLR5nQcqo7oWJf3VUMrryUfJfO7TGEkMyl8aOkbkzODob4SP/v6XhS83e/4WUXW4a9YlXd+ZC6qB+xPsNYw3OJJ/y";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // SWITCH TO FRONT-FACING CAMERA??
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        // load images (4) from asset
        imageTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        for (int i = 0; i < trackableTargets.length; i++)
        {
            trackableTargets[i] = imageTargets.get(i);
        }

        trackableTargets[0].setName("Wheels");
        trackableTargets[1].setName("Tools");
        trackableTargets[2].setName("Legos");
        trackableTargets[3].setName("Gears");

        // assign listener to each trackable target
        for (int i = 0; i < trackableTargets.length; i++)
        {
            listeners[i] = (VuforiaTrackableDefaultListener) trackableTargets[i].getListener();
        }

        // TODO: create a function to create a location matrix for target
        // set target locations
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix // blue right
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmFTCFieldWidth, 1524, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        trackableTargets[0].setLocation(wheelsTargetLocationOnField);
        //RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix // red right
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(2743.2f, mmFTCFieldWidth, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackableTargets[1].setLocation(toolsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix // red left
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(1524, mmFTCFieldWidth, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackableTargets[3].setLocation(gearsTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix // blue left
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth, 2743.2f, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        trackableTargets[2].setLocation(legosTargetLocationOnField);
        //RobotLog.ii(TAG, "Legos Target=%s", format(toolsTargetLocationOnField));


        // set phone location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(-19.05f, 203.0f, 3.0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));  // portrait phone

        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


        for (int i = 0; i < listeners.length; i++)
        {
            listeners[i].setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // We show the log in oldest-to-newest order, as that's better for poetry
        //telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        //telemetry.log().setCapacity(8);

        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start tracking");
        //telemetry.update();
    }

    void startTracking()
    {
        imageTargets.activate();
    }

    void stopTracking()
    {
        imageTargets.deactivate();
    }

    // TODO: make it tell us which target is visible
    public boolean isVisible(int targetIndex)
    {
        return listeners[targetIndex].isVisible();
    }

    public OpenGLMatrix getLocation(int targetIndex)
    {
        /**
         * getUpdatedRobotLocation() will return null if no new information is available since
         * the last time that call was made, or if the trackable is not currently visible.
         * getRobotLocation() will return null if the trackable is not currently visible.
         */

        //OpenGLMatrix robotLocationTransform = listeners[i].getUpdatedRobotLocation();
        OpenGLMatrix robotLocationTransform = listeners[targetIndex].getRobotLocation();
        if (robotLocationTransform != null)
        {
            lastLocation = robotLocationTransform;
            return robotLocationTransform; // could be in class
        }
        OpenGLMatrix location = null;
        return location;
    }


    //CodeReview: For GetBeaconColor(), it would be better to use enums to return defined/named values, rather than ints.
    //            An enum lets you define your own names for values.
    //            It will make your code easier to read and
    //            reduce the chances of accidental bugs.
    //            Here's a code sample

    /*
    enum BeaconColor {
        red,
        blue,
        undefined
    }

    public BeaconColor GetBeaconColor() throws InterruptedException {
        return BeaconColor.blue;
    }

    BeaconColor beaconColor = GetBeaconColor();

    if (beaconColor == BeaconColor.blue) { }

    */

    // return 0: blue, 1: red
    // get the image from Vuforia, project beacon point onto image
    // Gets the COLOR of the LEFT side of beacon
    public int GetBeaconColor() throws InterruptedException
    {
        for (int i = 0; i < listeners.length; i++) {
            if (listeners[i].isVisible())   // if tracking image
            {
                //takes the frame at the head of the queue
                frame = vuforia.getFrameQueue().take();
                image = null;
                imageRGB565 = null;
                imageRGB888 = null;

                long numImages = frame.getNumImages();

                for (int j = 0; j < numImages; j++) {
                    image = frame.getImage(j);
                    imageFormat = image.getFormat();

                    if (imageFormat == PIXEL_FORMAT.RGB565)
                    {
                        //telemetry.log().add("got image565");
                        break;
                    } // if
                } // for

                pose = listeners[i].getRawPose();
                if (pose != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);
                    // image size is 254 mm x 184 mm
                    // point to upper left
                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
                    Vec2F beaconLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127 + 85, 92 + 90, 0));
                    Vec2F beaconRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127 - 85, 92 + 90, 0));

                    GetImageColor(image, beaconLeft, leftColorHSV);
                    GetImageColor(image, beaconRight, rightColorHSV);  // for now, we're just looking at the left side
                    frame.close();  // close frame to free memory

                    // adjust color for red range (if red is between 0 and 45 degrees, shift by adding 300 so that red is greater than blue
                    float colorLeft = (leftColorHSV[0] < 45) ? leftColorHSV[0] + 300 : leftColorHSV[0];
                    float colorRight = (rightColorHSV[0] < 45) ? rightColorHSV[0] + 300 : rightColorHSV[0];

                    float deltaColorHSV = colorLeft - colorRight;
                    if (deltaColorHSV < 0) // if left color is negative, then left side is blue
                    {
                        return 0; // BLUE
                    } else {
                        return 1; // RED
                    }

                    //telemetry.log().add(String.format("LeftSideHue: %f RightSideHue: %f", leftColorHSV[0], rightColorHSV[0]));
                }
            }
        }
        return 2; // return illegal color value
    }


    // sample area in image to determine color hue for beacon - detect red or blue
    public void GetImageColor(Image image, Vec2F beaconPoint, float[] colorHsvOut) throws InterruptedException
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
            int x = (int) beaconPoint.getData()[0];
            int y = (int) beaconPoint.getData()[1];

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
            for (int k = 0; k < 3; k++) {
                colorHsvOut[k] = colorHsvSum[k] / 256;
            }
            //telemetry.log().add(String.format("Hue: %f, Sat: %f, Val: %f", colorHSV[0], colorHSV[1], colorHSV[2]));
        }

    }

    public float GetImageCenter (int targetIndex)
    {
        if (listeners[targetIndex].isVisible() )
        {
            pose = listeners[targetIndex].getRawPose();
            if (pose != null) {
                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);
                // image size is 254 mm x 184 mm
                // point to upper left
                Vec2F center = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));
                return center.getData()[1]; // return y coordinate of picture (horizontal if image because we're in portrait view
            }
        }
        return 10000; // return invalid value
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

}


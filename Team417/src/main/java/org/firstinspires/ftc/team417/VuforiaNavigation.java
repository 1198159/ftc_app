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
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    Image imageRGB888 = null;
    int imageFormat;
    Bitmap bm;      // android.graphics

    VuforiaTrackables imageTargets;
    /** For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    VuforiaTrackable Wheels = imageTargets.get(0);
    VuforiaTrackable Tools = imageTargets.get(1);
    VuforiaTrackable Legos = imageTargets.get(2);
    VuforiaTrackable Gears = imageTargets.get(3);
    OpenGLMatrix pose;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARvv/bz/////AAAAGVPnoD3yyERZqCnr7KHm3FgrFIn4fQGOfufF8iBOidgC9FrsCDLKKjDvkgVJybQpEu3brET0BeFbyb746Cu/gT8E6S89bGDF2h6iSlKk4IRoB60EPX/p+yTPMr82hZRxS5SGDlU1JVarJwhN3el/donXpG70LYWRrInA/51wa+pSkM64ZdhRcVwTutBBoXvFMdil6YBwmHTgPK8TpGLHBFYlJskNT2FxdGQiB5JyXkzNLoNSE3MeUDBsdZ0H2X39BOqcEh0hgZrLQ33WuQBwLR5nQcqo7oWJf3VUMrryUfJfO7TGEkMyl8aOkbkzODob4SP/v6XhS83e/4WUXW4a9YlXd+ZC6qB+xPsNYw3OJJ/y";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        imageTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        Wheels = imageTargets.get(0);
        Wheels.setName("Wheels");
        Tools = imageTargets.get(1);
        Tools.setName("Tools");
        Legos = imageTargets.get(2);
        Legos.setName("Legos");
        Gears = imageTargets.get(3);
        Gears.setName("Gears");


        allTrackables.addAll(imageTargets);

        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        Wheels.setLocation(wheelsTargetLocationOnField);
        //RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        Tools.setLocation(toolsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        Gears.setLocation(gearsTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        Legos.setLocation(legosTargetLocationOnField);
        //RobotLog.ii(TAG, "Legos Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0));  // portrait phone
        //AngleUnit.DEGREES, -90, 0, 0));  // landscape
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener) Wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) Tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) Gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) Legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        // We show the log in oldest-to-newest order, as that's better for poetry
        //telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        //telemetry.log().setCapacity(8);

        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start tracking");
        //telemetry.update();

        /** Start tracking the data sets we care about. */
        imageTargets.activate();
    }

    public boolean isVisible()
    {
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
            if ( ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() )
            {
                return true;
            }
        }
        return false;
    }

    public OpenGLMatrix getLocation() {
        // allTrackables is a list of trackable imageTargets
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();
            return robotLocationTransform; // could be in class
        }
        OpenGLMatrix location = null;
        return location;
    }

    public void updateLocation() throws InterruptedException {
        lastLocation = null;

        // allTrackables is a list of trackable imageTargets
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

            if (robotLocationTransform != null)
            {
                lastLocation = robotLocationTransform;
                GetImage(trackable);
            }
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                //telemetry.addData("Pos", format(lastLocation));
            } else {
                //telemetry.addData("Pos", "Unknown");
            }
            //telemetry.update();
            //idle();
        }
    }
    // get image from Vuforia, project beacon point onto image
    public void GetImage(VuforiaTrackable trackable) throws InterruptedException

    {
        frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        image = null;
        imageRGB565 = null;
        imageRGB888 = null;

        long numImages = frame.getNumImages();
        //telemetry.addData("numImages", String.format("%d", numImages));

        for (int i = 0; i < numImages; i++) {
            image = frame.getImage(i);
            imageFormat = image.getFormat();

            if (imageFormat == PIXEL_FORMAT.RGB888) {
                imageRGB888 = image;
                //telemetry.log().add("got image888");
                break;
            } else if (imageFormat == PIXEL_FORMAT.RGB565) {
                imageRGB565 = image;
                //telemetry.log().add("got image565");
                break;
            } // if
        } // for

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();
        if (pose != null)
        {
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            // point to upper left
            Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
            Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
            Vec2F beaconLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127 + 85, 92 + 90, 0));
            Vec2F beaconRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127 - 85, 92 + 90, 0));
            /*telemetry.log().add(String.format("UpperLeft X: %f, Y: %f", upperLeft.getData()[0], upperLeft.getData()[1]));
            telemetry.log().add(String.format("UpperRight X: %f, Y: %f", upperRight.getData()[0], upperRight.getData()[1]));
            telemetry.log().add(String.format("BeaconLeft X: %f, Y: %f", beaconLeft.getData()[0], beaconLeft.getData()[1]));
            telemetry.log().add(String.format("BeaconRight X: %f, Y: %f", beaconRight.getData()[0], beaconRight.getData()[1]));
            */

            float leftColorHSV[] = {0f, 0f, 0f};
            float rightColorHSV[] = {0f, 0f, 0f};

            GetImageColor(image, beaconLeft, leftColorHSV);
            GetImageColor(image, beaconRight, rightColorHSV);
            //telemetry.log().add(String.format("LeftSideHue: %f RightSideHue: %f", leftColorHSV[0], rightColorHSV[0]));
        }

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

            x = (x < imageWidth) ? x : imageWidth;
            y = (y < imageHeight) ? y : imageHeight;

            // create bitmap of image to detect color
            bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            //telemetry.log().add(String.format("Width: %d, Height: %d, Stride: %d", imageWidth, imageHeight, stride));

            // sample and integrate colors for a 16x16 square region of pixels
            for (int j = y - 8; j < y + 8; j++) {
                for (int i = x - 8; i < x + 8; i++) {
                    // get RGB color of pixel
                    color = bm.getPixel(i, j);

                    // convert RGB to HSV - hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    Color.colorToHSV(color, colorHSV);

                    // integrate HSV color of all pixels
                    for (int k = 0; k < 3; k++)
                        colorHsvSum[k] += colorHSV[k];

                    // draw black border around sample region for debugging only
                    if ((j == y - 8) || (j == y + 7) || (i == x - 8) || (i == x + 7)) {
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
        frame.close();  // close frame to free memory

    }

}

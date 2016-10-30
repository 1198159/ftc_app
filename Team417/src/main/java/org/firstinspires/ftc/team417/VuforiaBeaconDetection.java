/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.team417;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
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

import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Vuforia Test", group ="Concept")
//@Disabled
public class VuforiaBeaconDetection extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    Image imageRGB888 = null;
    int imageFormat;
    Bitmap bm;      // android.graphics
    boolean gamePadButtonA;  // determine which half of beacon to sample

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARvv/bz/////AAAAGVPnoD3yyERZqCnr7KHm3FgrFIn4fQGOfufF8iBOidgC9FrsCDLKKjDvkgVJybQpEu3brET0BeFbyb746Cu/gT8E6S89bGDF2h6iSlKk4IRoB60EPX/p+yTPMr82hZRxS5SGDlU1JVarJwhN3el/donXpG70LYWRrInA/51wa+pSkM64ZdhRcVwTutBBoXvFMdil6YBwmHTgPK8TpGLHBFYlJskNT2FxdGQiB5JyXkzNLoNSE3MeUDBsdZ0H2X39BOqcEh0hgZrLQ33WuQBwLR5nQcqo7oWJf3VUMrryUfJfO7TGEkMyl8aOkbkzODob4SP/v6XhS83e/4WUXW4a9YlXd+ZC6qB+xPsNYw3OJJ/y";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        // error vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        // VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackables imageTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable Wheels = imageTargets.get(0);
        Wheels.setName("Wheels");
        VuforiaTrackable Tools = imageTargets.get(1);
        Tools.setName("Tools");
        VuforiaTrackable Legos = imageTargets.get(2);
        Legos.setName("Legos");
        VuforiaTrackable Gears = imageTargets.get(3);
        Gears.setName("Gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(imageTargets);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        Wheels.setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        Tools.setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0));  // portrait phone
        //AngleUnit.DEGREES, -90, 0, 0));  // landscape
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) Wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) Tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(8);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        imageTargets.activate();

        while (opModeIsActive()) {

            lastLocation = null;

            // allTrackables is a list of trackable imageTargets
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null && (gamepad1.a || gamepad1.b))
//                if (gamepad1.a)
                {
                    gamePadButtonA = gamepad1.a;    // used later to determine which beacon half to sample
                    lastLocation = robotLocationTransform;
                    GetImage(trackable);
                }
            }


/*rgb is now the Image object that we’ve used in the video*/
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            idle();
        }
    }

    // get image from Vuforia, project beacon point onto image
    public void GetImage(VuforiaTrackable trackable) throws InterruptedException {
        frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        image = null;
        imageRGB565 = null;
        imageRGB888 = null;

        long numImages = frame.getNumImages();
        telemetry.addData("numImages", String.format("%d", numImages));

        for (int i = 0; i < numImages; i++) {
            image = frame.getImage(i);
            imageFormat = image.getFormat();

            if (imageFormat == PIXEL_FORMAT.RGB888) {
                imageRGB888 = image;
                telemetry.log().add("got image888");
                break;
            } else if (imageFormat == PIXEL_FORMAT.RGB565) {
                imageRGB565 = image;
                telemetry.log().add("got image565");
                break;
            }//if
        }//for

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();
        if (pose != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            // point to upper left
            Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
            Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
            Vec2F beaconLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127 + 50, 92 + 60, 0));
//            Vec2F beaconLeft = new Vec2F(640, 360);

            Vec2F beaconRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127 - 50, 92 + 60, 0));
            telemetry.log().add(String.format("UpperLeft X: %f, Y: %f", upperLeft.getData()[0], upperLeft.getData()[1]));
            telemetry.log().add(String.format("UpperRight X: %f, Y: %f", upperRight.getData()[0], upperRight.getData()[1]));
            telemetry.log().add(String.format("BeaconLeft X: %f, Y: %f", beaconLeft.getData()[0], beaconLeft.getData()[1]));
            telemetry.log().add(String.format("BeaconRight X: %f, Y: %f", beaconRight.getData()[0], beaconRight.getData()[1]));


            float colorHSV[] = {0f, 0f, 0f};
            if (gamePadButtonA)
                GetImageColor(image, beaconLeft, colorHSV);
            else
                GetImageColor(image, beaconRight, colorHSV);

        }

    }

    // sample area in image to determine color hue for beacon - detect red or blue
    public void GetImageColor(Image image, Vec2F beaconPoint, float[] colorHsvOut) throws InterruptedException {

        if (image != null) {

            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            int stride = image.getStride();
            int color = 0;
            int colorRed = 0;
            int colorGreen = 0;
            int colorBlue = 0;
            byte pixel0;
            byte pixel1;
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

            telemetry.log().add(String.format("Width: %d, Height: %d, Stride: %d", imageWidth, imageHeight, stride));

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
            telemetry.log().add(String.format("Hue: %f, Sat: %f, Val: %f", colorHSV[0], colorHSV[1], colorHSV[2]));
        }
        frame.close();  // close frame to free memory

    }


    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}

/* obsolete code that was used in previous versions
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);

                    pixel0 = pixelArray[j * stride + i * 2];
                    pixel1 = pixelArray[j * stride + i * 2 + 1];

                    // extract color from RGB565 format pixel
                    // RGB565 format: R-5, G-6, B-5
                    // pixel1 7:3 Red[4:0]
                    // Pixel1 2:0 Green[5:3]
                    // Pixel0 7:5 Green[2:0]
                    // Pixel0 4:0 Blue[4:0]

                    colorRed = (byte) ((pixel1 & 0xF8) >> 3);
                    colorGreen = (byte) (((pixel1 & 0x07) << 3) | ((pixel0 & 0xE0) >> 5));
                    colorBlue = (byte) (pixel0 & 0x1F);
                    // integrate individual colors
                    //colorRed += (pixel1 & 0xF8);
                    //colorGreen += (((pixel1 & 0x07) << 3) | ((pixel0 & 0xE0) >> 5));
                    //colorBlue += (pixel0 & 0x1F);

                    colorRed <<= 3;
                    colorGreen <<= 2;
                    colorBlue <<= 3;

                    // convert RGB to hue, sat, val
                    // hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
                    //Color.RGBToHSV(colorRed, colorGreen, colorBlue, colorHSV);
//            colorRed >>= 3;  // adjust for bit position 7:3 above
//            telemetry.log().add(String.format("Red: %d, Green: %d, Blue: %d", colorRed, colorGreen, colorBlue));
//            colorRed >>= 8;  // adjust for 256 sum
//            colorGreen >>= 8;
//            colorBlue >>= 8;

//            Color.RGBToHSV(colorRed, colorGreen, colorBlue, colorHSV);

*/
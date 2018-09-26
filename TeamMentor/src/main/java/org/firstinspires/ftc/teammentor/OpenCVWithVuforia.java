/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teammentor;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * This OpMode illustrates the basics of using opencv with an image obtained using Vuforia.
 *
 */

@TeleOp(name="opencv vuforia", group ="opencv")
//@Disabled
public class OpenCVWithVuforia extends LinearOpMode {

    public static final String TAG = "opencv with vuforia Sample";

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        
        //This licence key belongs to Steve Geffner
        parameters.vuforiaLicenseKey = "ATJf0AL/////AAAAGQZ9xp9L+k5UkmHj3LjxcoQwNTTBJqjO9LYsbkWQArRpYKQmt7vqe680RCQSS9HatStn1XZVi7rgA8T7qrJz/KYI748M4ZjlKv4Z11gryemJCRA9+WWkQ51D3TuYJbQC46+LDeMfbvcJQoQ79jtXr7xdFhfJl1mRxf+wMVoPWfN6Dhr8q3XVxFwOE/pM3gXWQ0kacbcGR/vy3NAsbOhf02DEe5WoV5PNZTF34LWN3dWURu7NJsnbFzkpzXdogeVAdiQ3QUWDvuhEwvSJY4W+fCTb15t6T/c/GJ/vqptsVKqavXk6MQobnUsVFpFP+5OSuRQe7EgvWuOxn7xn5YlC+CWAYh9LrXDpktwCwBAiX3Gx";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        // Setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        // Make sure that vuforia doesn't begin racking up unnecessary frames
        vuforia.setFrameQueueCapacity(1);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean xButtonAlreadyDown = false;

        while (opModeIsActive()) {

            if (gamepad1.x & !xButtonAlreadyDown) {
                xButtonAlreadyDown = true; //debounce the button so this code only runs one time per button press

                Bitmap bitmap = getBitmapFromVuforia();
                Mat mat  = getMatFromBitmap(bitmap);
                if (mat != null)
                {
                    //do opencv operations here

                }

            }
            else //reset our debounce variable
                if (!gamepad1.x & xButtonAlreadyDown) xButtonAlreadyDown = false;


            telemetry.update();
        }
    }

    public Bitmap getBitmapFromVuforia()
    {
        VuforiaLocalizer.CloseableFrame frame = null; //used for storing info from vuforia frame queue to be analyzed in other methods
        Image image = null;  //an image that we will obtain from vuforia
        Bitmap bitMap = null; //a bitmap representation of that image
        int imageFormat = 0; //the format of the received image
        long numImages = 0; //the number of images obtained
        boolean goodImageFound = false;

        try { frame = vuforia.getFrameQueue().take(); }
        catch (Exception e) {return null;}

        if (frame==null) return null;

        numImages = frame.getNumImages();
        if (numImages==0) return null;

        for (int j = 0; j < numImages; j++)
        {
            image = frame.getImage(j);
            imageFormat = image.getFormat();

            if (imageFormat == PIXEL_FORMAT.RGB565) {
                goodImageFound = true;
                break;
            }
        }

        if (!goodImageFound) return null;

        int imageWidth = image.getWidth();
        int imageHeight = image.getHeight();

        // Create bitmap of image to detect color
        bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bitMap.copyPixelsFromBuffer(image.getPixels());

        //close the frame, prevents memory leaks and crashing
        frame.close();

        return bitMap;
    }

    public Mat getMatFromBitmap(Bitmap b)
    {
        if (b==null) return null;

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(b.getWidth(), b.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(b, tmp);

        return tmp;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

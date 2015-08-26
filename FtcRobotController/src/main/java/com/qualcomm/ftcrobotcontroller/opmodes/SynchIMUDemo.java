package com.qualcomm.ftcrobotcontroller.opmodes;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IAdaFruitBNO055IMU;
import org.swerverobotics.library.internal.AdaFruitBNO055IMU;

/**
 * SynchIMUDemo gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 * http://www.adafruit.com/products/2472
 */
public class SynchIMUDemo extends SynchronousOpMode
    {
    IAdaFruitBNO055IMU imu;
    
    @Override public void main() throws InterruptedException
        {
        imu = AdaFruitBNO055IMU.create(hardwareMap.i2cDevice.get("imu"));

        waitForStart();
        
        while (opModeIsActive())
            {
            
            
            telemetry.dashboard.update();
            idle();
            }
        }
    }

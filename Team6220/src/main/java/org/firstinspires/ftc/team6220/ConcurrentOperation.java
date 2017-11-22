package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.HardwareMap;

/*
    Represents an object that acts concurrently to other functions over multiple cycles.
    Objects from this interface are added to an array that will call them appropriately.
*/
public interface ConcurrentOperation
{
    // Robot hardware and software setup called once before match
    public void initialize(HardwareMap hMap);

    // Called at the end of each cycle
    public void update(double eTime);
}

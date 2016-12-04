package org.firstinspires.ftc.team6220;

/*
    Represents an object that acts concurrently to other functions over multiple cycles.
    Objects from this interface are added to an array that will call them apropriately.

*/
public interface ConcurrentOperation
{
    //called once at startup
    public void initialize();

    //called at the end of each cycle
    public void update(double eTime);

}

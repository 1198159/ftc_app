package org.firstinspires.ftc.team6220;

/*
    For PID controlled systems, this enum dictates what states are enforced.
*/

//TODO should these names be changed to be more specific to our uses?
public enum PIDEnforcementMode
{
    NONE,                //do not use any PID control
    DERIVATIVE,          //target is a rate of change, e.g. a velocity in a particular direction
    HIGHEST_ORDER,       //target is of the highest non-lagrangian order, e.g. a position in a particular direction

    //this may not be useful, since we can use highest order to achieve identical results
    //DERIVATIVE_INTEGRAL, //target is a rate of change, but enforces an average velocity instead of instantaneous velocity by calculating the highest non-lagrangian ordered value
}

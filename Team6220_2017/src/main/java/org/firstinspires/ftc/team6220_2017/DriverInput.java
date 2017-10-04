package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
    Encapsulates gamepad for interpretation of control actions
*/
public class DriverInput implements ConcurrentOperation
{
    private Gamepad controller;     //one state for each button; there are 13 buttons available
    private boolean[] buttonStates = {false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    private boolean[] lastButtonStates = {false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    private double[] buttonHeldCounts = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};


    public DriverInput(Gamepad cont)
    {
        controller = cont;
    }

    public void initialize(HardwareMap hMap){}

    public boolean isButtonPressed(Button label)
    {
        return buttonStates[label.ordinal()];
    }

    public boolean isButtonJustPressed(Button label)
    {
        return buttonStates[label.ordinal()] && !lastButtonStates[label.ordinal()];
    }

    public boolean isButtonJustReleased(Button label)
    {
        return !buttonStates[label.ordinal()] && lastButtonStates[label.ordinal()];
    }

    public double getButtonHoldTime(Button label)
    {
        return buttonHeldCounts[label.ordinal()];
    }

    public double getLeftStickAngle()
    {
        return Math.atan2(-controller.left_stick_y, controller.left_stick_x);
    }

    public double getRightStickAngle()
    {
        return Math.atan2(-controller.right_stick_y, controller.right_stick_x);
    }

    public double getLeftStickMagnitude()
    {
        return Math.sqrt(Math.pow(controller.left_stick_x, 2) + Math.pow(controller.left_stick_y, 2));
    }

    public double getRightStickMagnitude()
    {
        return Math.sqrt(Math.pow(controller.right_stick_x, 2) + Math.pow(controller.right_stick_y, 2));
    }

    //call at end of loop
    public void update(double eTime)
    {
        for(int i=0; i < 14; i++)
        {
            lastButtonStates[i] = buttonStates[i];
        }
        buttonStates[0] = controller.a;
        buttonStates[1] = controller.b;
        buttonStates[2] = controller.x;
        buttonStates[3] = controller.y;
        buttonStates[4] = controller.left_bumper;
        buttonStates[5] = controller.right_bumper;
        buttonStates[6] = controller.left_stick_button;
        buttonStates[7] = controller.right_stick_button;
        buttonStates[8] = controller.back;
        buttonStates[9] = controller.start;
        buttonStates[10] = controller.dpad_up;
        buttonStates[11] = controller.dpad_down;
        buttonStates[12] = controller.dpad_left;
        buttonStates[13] = controller.dpad_right;

        for (int i = 0; i < 14; i++)
        {
            if(buttonStates[i])
            {
                buttonHeldCounts[i] += eTime;
            }
            else
            {
                buttonHeldCounts[i] = 0;
            }
        }
    }

}

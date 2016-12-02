package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
    Encapsulates gamepad for interpretation of control actions
*/
public class DriverInput
{
    private Gamepad controller;
    private boolean[] buttonStates = {false,false,false,false,false,false,false,false,false,false};
    private boolean[] lastButtonStates = {false,false,false,false,false,false,false,false,false,false};
    private double[] buttonHeldCounts = {0,0,0,0,0,0,0,0,0,0};

    private final String[] BUTTONS = {"A","B","X","Y",
            "Left Bumper","Right Bumper","Left Stick Press","Right Stick Press",
            "Select","Start"};

    public DriverInput(Gamepad cont)
    {
        controller = cont;
    }


    //CodeReview: Instead of using strings to represent the buttons, how about using an enum?
    //            Then you wouldn't need to map strings to integer positions in the arrays.
    //            You can define an integer value for each item in the enum:
    //   enum Buttons {
    //        A(0),
    //        B(1),
    //        public int value;
    //        Buttons(int i) value = i }
    //   }
    //
    //  ...then you can do something like this:
    //  isButtonPressed(Buttons.A)
    //  ...or this:
    //  myArray[Buttons.A.value]


    //TODO replace with hashmap or something
    private int getButtonIndexFromString(String label) {
        int index = 10;
        for (int i=0;i<10;i++)
        {
            if (BUTTONS[i] == label)
            {
                index = i;
            }
        }

        return index;
    }

    public boolean isButtonPressed(String label)
    {
        return buttonStates[getButtonIndexFromString(label)];
    }

    public boolean isButtonJustPressed(String label)
    {
        int index = getButtonIndexFromString(label);
        return buttonStates[index] && !lastButtonStates[index];
    }

    public boolean isButtonJustReleased(String label)
    {
        int index = getButtonIndexFromString(label);
        return !buttonStates[index] && lastButtonStates[index];
    }

    public double getButtonHoldTime(String label)
    {
        return buttonHeldCounts[getButtonIndexFromString(label)];
    }

    //TODO check if y axes are actually flipped
    public double getLeftStickAngle()
    {
        return Math.atan2(-controller.left_stick_y,controller.left_stick_x);
    }

    public double getRightStickAngle()
    {
        return Math.atan2(-controller.right_stick_y,controller.right_stick_x);
    }

    public double getLeftStickMagnitude()
    {
        return Math.pow(Math.pow(controller.left_stick_x, 2) + Math.pow(controller.left_stick_y, 2), 0.5);
    }

    public double getRightStickMagnitude()
    {
        return Math.pow(Math.pow(controller.right_stick_x,2)+Math.pow(controller.right_stick_y,2),0.5);
    }

    //call at start of loop
    public void update(double eTime)
    {
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

        for (int i = 0; i<10;i++)
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

    //call at end of loop
    public void roll()
    {
        for(int i=0;i<10;i++)
        {
            lastButtonStates[i] = buttonStates[i];
        }
    }

}

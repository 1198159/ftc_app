package org.firstinspires.ftc.team6220;

/*
    This class contains functions that are used to process 1D data sets.
*/

public abstract class SequenceUtilities {

    //return all the values of a 1D array added together
    public static double sum(double[] list)
    {
        double total = 0;
        for (int i=0; i<list.length; i++ ){
            total += list[i];
        }
        return total;
    }

    //return the mean values of a list
    public static double average(double[] list)
    {
        double total = sum(list);
        return total / list.length;
    }

    //return a weighted average of a list, with weights assigned per value by a seconds array of the same size
    public static double weightedAverage(double[] list, double[] w)
    {
        double total = 0;
        for (int i=0; i<list.length; i++ ){
            total += list[i]*w[i];
        }
        return total / sum(w);
    }
}

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
    public static double average(double[] array)
    {
        double total = sum(array);
        return total / array.length;
    }

    //return a weighted average of a list, with weights assigned per value by a seconds array of the same size
    public static double weightedAverage(double[] array, double[] w)
    {
        double total = 0;
        for (int i=0; i<array.length; i++ ){
            total += array[i]*w[i];
        }
        return total / sum(w);
    }

    //shifts an array down one place and puts a new value in the new "empty" slot, at array[0]
    public static double[] roll(double[] array, double newValue)
    {
        for (int i=(array.length-2); i>=0; i-- ){
            array[i+1] = array[i];
        }
        array[0] = newValue;
        return array;
    }

    //shifts an array up one place and puts a new value in the new "empty" slot, ar array[n-1]
    public static double[] rollReversed(double[] array, double newValue)
    {
        for (int i=0; i<(array.length-2); i++ ){
            array[i] = array[i+1];
        }
        array[array.length-1] = newValue;
        return array;
    }

    //get the largest value
    //NOTE: negative values are considered smaller than any positive values
    public static double getLargestValue(double[] array)
    {
        int size = array.length;
        double largest = array[0];
        for(int i = 0; i < size-1; i++ )
        {
            largest = Math.max(largest,array[i+1]);
        }
        return largest;

    }

    //get the largest magnitude
    public static double getLargestMagnitude(double[] array)
    {
        int size = array.length;
        double largest = array[0];
        for(int i = 0; i < size-1; i++ )
        {
            largest = Math.max(Math.abs(largest), Math.abs(array[i + 1]));
        }
        return largest;
    }

    //return the sum of two tuples as vectors
    public static double[] vectorAdd(double[] a, double[] b)
    {
        int size = a.length;
        double[] c = new double[size];
        for(int i = 0; i < size; i++ )
        {
            c[i] = a[i] + b[i];
        }
        return c;
    }

}

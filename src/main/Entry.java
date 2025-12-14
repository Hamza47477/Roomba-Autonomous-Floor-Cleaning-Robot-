package main;

/**
 * Created by Theo Theodoridis.
 * Class    : Entry
 * Version  : v1.0
 * Date     : © Copyright 2015
 * User     : ttheod
 * email    : t.theodoridis@salford.ac.uk
 * Comments : Implementation of a input/output data entry class to be used in FSMs.
 **/

public class Entry
{
    public double data[];
    public boolean block;

    public Entry(double data[], boolean block)
    {
        this.data = data;
        this.block = block;
    }
}

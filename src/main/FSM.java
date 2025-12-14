package main;

/**
 * Created by Theo Theodoridis.
 * Class    : FSM
 * Version  : v1.0
 * Date     : © Copyright 2015
 * User     : ttheod
 * email    : t.theodoridis@salford.ac.uk
 * Comments : Implementation of a Finite State Machine.
 **/

public interface FSM
{
    public int getPriority();
    public double[] getSensors();
    public boolean getInhibitor();
    public boolean getSuppressor();

    public void setSensors(double sensor[]);
    public void setInhibitor(boolean inhibitor);
    public void setSuppressor(boolean suppressor);

    public void run();
    public int update(Entry inputEntry[], Entry outputEntry[], boolean reset);
}

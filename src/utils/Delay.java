package utils;

/**
 * Created by Theodoros Theodoridis.
 * Class    : Delay
 * Version  : v1.0
 * Date     : © Copyright 22-Mar-2010
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

public class Delay
{
   /**
    * Method     : Delay::ms()
    * Purpose    : To delay in milliseconds (ms).
    * Parameters : - ms : The milliseconds scalar.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public static void ms(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(Exception e)
        {
            System.out.println("Exception<Delay>: " + e);
        };
    }

   /**
    * Method     : Delay::sec()
    * Purpose    : To delay in seconds (sec).
    * Parameters : - sec : The seconds scalar.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public static void sec(int sec)
    {
        ms(sec * 1000);
    }

   /**
    * Method     : Delay::min()
    * Purpose    : To delay in minutes (min).
    * Parameters : - min : The minutes scalar.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public static void min(int min)
    {
        sec(min * 60);
    }
}

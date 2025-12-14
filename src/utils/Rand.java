package utils;

import java.util.Random;

/**
 * Created by Theo Theodoridis.
 * Class    : Rand
 * Version  : v1.0
 * Date     : © Copyright 08-Oct-2008
 * User     : Theodore
 * email    : ttheod@gmail.com
 * Comments :
 **/

public class Rand
{
    private static Random rand = new java.util.Random();

   /**
    * Method     : Rand::getBool()
    * Purpose    : To generate a boolean random number from the interval [0, 1].
    * Parameters : None.
    * Returns    : A boolean random number.
    * Notes      : None.
    **/
    public static boolean getBool()
    {
        return(rand.nextBoolean());
    }

   /**
    * Method     : Rand::getInt()
    * Purpose    : To generate an integer random number from the interval [from, to].
    * Parameters : - from : The >= number starting the generator.
    *              - from : The <= number stopping the generator.
    * Returns    : An integer random number.
    * Notes      : None.
    **/
    public static int getInt(int from, int to)
    {
        return(from + Math.abs(rand.nextInt()) % (Math.abs(from - to) + 1));
    }

   /**
    * Method     : Rand::getDouble()
    * Purpose    : To generate a double random number from the interval [0, 1].
    * Parameters : None.
    * Returns    : ~[0, 1].
    * Notes      : None.
    **/
    public static double getDouble()
    {
        return(Math.random());
    }
}

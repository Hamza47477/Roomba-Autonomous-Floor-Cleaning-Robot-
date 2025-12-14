package utils;

import java.text.DecimalFormat;

/**
 * Created by Theo Theodoridis.
 * Class    : Utils
 * Version  : v1.0
 * Date     : © Copyright 02-Nov-2008
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

public class Utils
{
   /**
    * Method     : Utils::decimal()
    * Purpose    : To convert a float number with .0 (1) floating points.
    * Parameters : - number    : The float number.
    *              - precision : The floating point precision (ex: "0.0", "0.00", etc).
    * Returns    : The converted .0 float number.
    * Notes      : None.
    **/
    public static double getDecimal(double number, String precision)
    {
        double x = number;
        try
        {
            x = Double.parseDouble(new DecimalFormat(precision).format(number));
        }
        catch(NumberFormatException e) { }
        return(x);
    }

   /**
    * Method     : Utils::map()
    * Purpose    : To map a value from one set to another.
    * Parameters : - x        : The actual value of set A.
    *              - fromSetA : Start value of set A.
    *              - toSetA   : End value of set A.
    *              - fromSetB : Start value of set B.
    *              - toSetB   : End value of set B.
    * Returns    : The mapped value.
    * Notes      : None.
    **/
    public static double map(double x, double fromSetA, double toSetA, double fromSetB, double toSetB)
    {
        return((x - fromSetA) * (toSetB - fromSetB) / (toSetA - fromSetA) + fromSetB);
    }

   /**
    * Method     : Utils::getEuclidean()
    * Purpose    : To get the Euclidean distance.
    * Parameters : - x1 : The 1st x reference.
    *              - x2 : The 2nd x reference.
    *              - y1 : The 1st y reference.
    *              - y2 : The 2nd y reference.
    * Returns    : The distance.
    * Notes      : None.
    **/
    public static double getEuclidean(double x1, double y1, double x2, double y2)
    {
        return(Math.sqrt(Math.pow(x1 - x2, 2.0) + Math.pow(y1 - y2, 2.0)));
    }
}

package utils;

import java.util.TimerTask;

/**
 * Created by Theo Theodoridis.
 * Class    : Timer
 * Version  : v2.0
 * Date     : © Copyright 12-May-2016
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

public class Timer
{
    private volatile int time    = 0;
    private volatile int counter = 0;
    private volatile boolean timerFlag   = true;
    private volatile boolean restartFlag = true;
    private java.util.Timer timer = new java.util.Timer();

   /**
    * Method     : Timer::Timer()
    * Purpose    : Default Timer class constructor.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public Timer() { }

   /**
    * Method     : Timer::task
    * Purpose    : The timer task.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    private TimerTask task = new TimerTask()
    {
        @Override
        public void run()
        {
            if(timerFlag)
            {
                // Validate and setup restart:
                if(restartFlag)
                {
                    restartFlag = false;
                    return;
                }

                // Run the timer setup:
                if(counter == time)
                {
                    stop();
                    return;
                }
                counter++;
            }
        }
    };

   /**
    * Method     : Timer::start()
    * Purpose    : To start the timer.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : The scheduleAtFixedRate method runs the task thread for 1 ms and starts running after 0 ms.
    **/
    public void start()
    {
        timer.scheduleAtFixedRate(task, 0, 1);
    }

   /**
    * Method     : Timer::stop()
    * Purpose    : To stop the timer.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public synchronized void stop()
    {
        counter = 0;
        timerFlag = false;
    }

   /**
    * Method     : Timer::restart()
    * Purpose    : To restart the timer.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public synchronized void restart()
    {
        counter = 0;
        timerFlag = true;
        restartFlag = true;
    }

   /**
    * Method     : Timer::suspend()
    * Purpose    : To suspend the timer.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public synchronized void suspend()
    {
        timerFlag = false;
    }

   /**
    * Method     : Timer::resume()
    * Purpose    : To resume the timer.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public synchronized void resume()
    {
        timerFlag = true;
    }

   /**
    * Method     : Timer::getState(), setState()
    * Purpose    : To get/set the timer's state.
    * Parameters : - timerFlag : The timer flag.
    * Returns    : True if timer has elapsed, False otherwise.
    * Notes      : None.
    **/
    public synchronized boolean getState()               { return(timerFlag);          }
    public synchronized void setState(boolean timerFlag) { this.timerFlag = timerFlag; }

   /**
    * Method     : Timer::setMs(), setSec(), setMin()
    * Purpose    : To set the ms/sec/min.
    * Parameters : ms/sec/min : The time to count.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public synchronized void setMs(int ms)     { time = ms;         }
    public synchronized void setSec(int sec)   { setMs(sec * 1000); }
    public synchronized void setMin(int min)   { setSec(min * 60);  }

   /**
    * Method     : Timer::getMs(), getSec(), getMin()
    * Purpose    : To get the ms/sec/min time count.
    * Parameters : None.
    * Returns    : The ms/sec/min time.
    * Notes      : None.
    **/
    public synchronized int getMs()  { return(counter);        }
    public synchronized int getSec() { return(getMs() / 1000); }
    public synchronized int getMin() { return(getSec() / 60);  }
}

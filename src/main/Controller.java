package main;

import coppelia.*;

import org.bytedeco.opencv.opencv_core.CvPoint;
import org.bytedeco.opencv.opencv_core.CvScalar;
import org.bytedeco.opencv.opencv_core.IplImage;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacv.Java2DFrameUtils;

import javafx.fxml.FXML;
import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.image.PixelWriter;

import utils.Delay;
import utils.ImageViewer;
import utils.Timer;
import utils.Utils;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.io.PrintWriter;
import java.io.FileWriter;

import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgcodecs.IMREAD_GRAYSCALE;
import static org.bytedeco.opencv.global.opencv_imgproc.cvMatchTemplate;
import static org.bytedeco.opencv.global.opencv_imgproc.CV_TM_CCOEFF_NORMED;
import static org.bytedeco.opencv.global.opencv_imgproc.cvRectangle;

/**
 * Controller v4.0 — Hybrid Encoder+GPS Heading, GPS Validity Guard
 *
 * TWO BUGS FIXED vs v3:
 *
 * BUG 1 — GPS cold-start Long.MAX_VALUE overflow
 *   V-REP returns garbage (≈9.2e16) on the first few GPS reads before
 *   the simulation socket is ready. v3 used this value to seed the
 *   heading calibration and waypoint selection, corrupting everything.
 *   FIX: isGpsValid() rejects |x| or |y| > 4 m. CLEAN holds still
 *   (setVel 0,0) until GPS settles to a sane room coordinate.
 *
 * BUG 2 — Heading estimator deadlock (spin forever, never translate)
 *   v3 used GPS movement to estimate heading, but heading is required
 *   before the robot will move → GPS never changes → heading never
 *   updates → robot spins forever at 146° without progressing.
 *   FIX: HYBRID estimator:
 *     PRIMARY  = encoder differential integration (works during spin)
 *     SECONDARY = GPS bearing calibration (snaps drift every ≥5 cm)
 *   The encoder integration updates headingEst every tick regardless
 *   of whether the robot is spinning or translating, so the heading
 *   error naturally decreases during the spin and the robot starts
 *   moving forward — no deadlock possible.
 */
public class Controller {

    @FXML private Button btnConnect;
    @FXML private Button btnAutoMode;
    @FXML private Button btnRight, btnLeft, btnBack, btnForward, btnStop;
    @FXML private Canvas canvasCamera;
    @FXML PixelWriter pw;
    @FXML private Label lblSensor0, lblSensor1, lblSensor2, lblSensor3, lblSensor4, lblSensor5;
    @FXML private Label lbl, lblGpsX, lblGpsY, lblGpsZ, lblRightWheel, lblLeftWheel;
    @FXML private Label lblBattery, lblBatteryTime, lblBehavior, lblDistance;

    private String defaultButtonStyle;

    private Color   imageCamera[][];
    private double  gpsValues[]     = new double[3];
    private double  sonarValues[]   = new double[6];
    private double  encoderValues[] = new double[2];

    private boolean runGPS = true, runCamera = true, runMotion = true;
    private boolean runSensors = true, runWheelEncoder = true;

    private IntW cameraHandle = new IntW(1), leftWheelHandle = new IntW(1), rightWheelHandle = new IntW(1);

    private boolean running = false, firstCameraRead = true;
    private boolean firstSensor0Read=true,firstSensor1Read=true,firstSensor2Read=true;
    private boolean firstSensor3Read=true,firstSensor4Read=true,firstSensor5Read=true;
    private boolean firstLeftWheelCall=true, firstRightWheelCall=true;

    private char dir = 's';
    private int  vel = 5;

    private double targetMinScore=0, targetMaxScore=0;
    private int resolutionCamera = 256;
    private CvPoint point = new CvPoint();
    private IntWA   resolution = new IntWA(1);
    private CharWA  image = new CharWA(resolutionCamera*resolutionCamera*3);
    private char    imageArray[] = new char[resolutionCamera*resolutionCamera*3];
    private Color   colorMatrix[][] = new Color[resolutionCamera][resolutionCamera];
    private BufferedImage bufferedImage = new BufferedImage(resolutionCamera,resolutionCamera,BufferedImage.TYPE_INT_RGB);

    private float dxRight=0,dxLeft=0;
    private float totalRightJointPosition=0,currentRightJointPosition=0,previousRightJointPosition=0;
    private float totalLeftJointPosition=0, currentLeftJointPosition=0, previousLeftJointPosition=0;
    private FloatW robotRightJointPosition = new FloatW(3);
    private FloatW robotLeftJointPosition  = new FloatW(3);

    public static final double CHARGER_XCOORD = 1.78;
    public static final double CHARGER_YCOORD = -0.78;

    private remoteApi vRep = new remoteApi();
    private int clientID = -1;

    // MAX_BATT_TIME = 750s → battery drains from 100% to 0% in 12.5 min
    // → reaches 20% at exactly 10 minutes (600s), matching assignment spec.
    // Formula: pct = (1 - t/MAX_BATT_TIME)*100  →  20 = (1-600/750)*100 ✓
    private int   MAX_BATT_TIME = 750;
    private final int MAX_BATT_VOLT = 12;
    private Timer motionTimer = new Timer();
    private Timer batteryTimer = new Timer();

    // ==========================================
    //  Battery
    // ==========================================
    // chargeOffset: added to raw timer reading to simulate charging.
    // While docked, this grows at CHARGE_RATE_PER_SEC per second, effectively
    // reversing the battery drain → getBatteryCapacity increases toward 100%.
    private double chargeOffset = 0.0;
    private static final double CHARGE_RATE_PER_SEC = 750.0 / 10.0; // full charge in 10s wall-clock

    public int     getBatteryTime()     { return batteryTimer.getSec(); }
    public boolean getBatteryState()    { return getBatteryCapacity() > 0.0; }
    public void    setBatteryTime(int min) {
        MAX_BATT_TIME = 60*min; motionTimer.setSec(MAX_BATT_TIME); motionTimer.restart();
    }
    public double getBatteryCapacity() {
        // Subtract chargeOffset from the elapsed seconds so that charging
        // appears to "rewind" the timer, increasing remaining capacity.
        double effectiveSec = Math.max(0.0, batteryTimer.getSec() - chargeOffset);
        double v = MAX_BATT_VOLT - Utils.map(effectiveSec, 0, (double)MAX_BATT_TIME, 0, (double)MAX_BATT_VOLT);
        return Math.max(0.0, v);
    }
    public double getBatteryPercentage() {
        return Math.max(0, Math.min(100, Math.round((getBatteryCapacity()/MAX_BATT_VOLT)*1000.0)/10.0));
    }

    // ==========================================
    //  Encoders
    // ==========================================
    private double readRightWheelEnc() {
        if (firstRightWheelCall) {
            vRep.simxGetJointPosition(clientID,rightWheelHandle.getValue(),robotRightJointPosition,remoteApi.simx_opmode_streaming);
            currentRightJointPosition=previousRightJointPosition=robotRightJointPosition.getValue();
            firstRightWheelCall=false; totalRightJointPosition=0;
        } else {
            vRep.simxGetJointPosition(clientID,rightWheelHandle.getValue(),robotRightJointPosition,remoteApi.simx_opmode_buffer);
            currentRightJointPosition=robotRightJointPosition.getValue();
            dxRight=getAngleMinusAlpha(currentRightJointPosition,previousRightJointPosition);
            totalRightJointPosition+=dxRight;
        }
        previousRightJointPosition=currentRightJointPosition;
        return Math.round((totalRightJointPosition/(2*Math.PI))*100d)/100d;
    }
    private double readLeftWheelEnc() {
        if (firstLeftWheelCall) {
            vRep.simxGetJointPosition(clientID,leftWheelHandle.getValue(),robotLeftJointPosition,remoteApi.simx_opmode_streaming);
            currentLeftJointPosition=previousLeftJointPosition=robotLeftJointPosition.getValue();
            firstLeftWheelCall=false; totalLeftJointPosition=0;
        } else {
            vRep.simxGetJointPosition(clientID,leftWheelHandle.getValue(),robotLeftJointPosition,remoteApi.simx_opmode_buffer);
            currentLeftJointPosition=robotLeftJointPosition.getValue();
            dxLeft=getAngleMinusAlpha(currentLeftJointPosition,previousLeftJointPosition);
            totalLeftJointPosition+=dxLeft;
        }
        previousLeftJointPosition=currentLeftJointPosition;
        return Math.round((totalLeftJointPosition/(2*Math.PI))*100d)/100d;
    }
    private float getAngleMinusAlpha(float a, float b) {
        double s=Math.sin(a)*Math.cos(b)-Math.cos(a)*Math.sin(b);
        double c=Math.cos(a)*Math.cos(b)+Math.sin(a)*Math.sin(b);
        return (float)Math.atan2(s,c);
    }
    public double getLeftWheelEnc()  { return encoderValues[0]; }
    public double getRightWheelEnc() { return encoderValues[1]; }
    public int    getEncoderNo()     { return 2; }

    // ==========================================
    //  GPS
    // ==========================================
    public double[] readGPS() {
        IntW bh=new IntW(1); FloatWA pos=new FloatWA(3);
        vRep.simxGetObjectHandle(clientID,"Roomba",bh,remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectPosition(clientID,bh.getValue(),-1,pos,remoteApi.simx_opmode_streaming);
        double[] p=new double[3];
        p[0]=Math.round((double)pos.getArray()[0]*100.0)/100.0;
        p[1]=Math.round((double)pos.getArray()[1]*100.0)/100.0;
        p[2]=Math.round((double)pos.getArray()[2]*100.0)/100.0;
        return p;
    }
    public double getGPSX()  { return gpsValues[0]; }
    public double getGPSY()  { return gpsValues[1]; }
    public double getGPSZ()  { return gpsValues[2]; }
    public int    getGPSNo() { return 3; }

    /**
     * Validates GPS. The room is ±2.15 m. V-REP returns ~9.2e16 on cold start
     * (Long.MAX_VALUE / 100 overflow). Reject anything outside ±4 m.
     */
    private boolean isGpsValid(double x, double y) {
        return Math.abs(x) < 4.0 && Math.abs(y) < 4.0;
    }

    // ==========================================
    //  Sonar
    // ==========================================
    private double readSonarRange(int sensor) {
        BoolW det=new BoolW(false); FloatWA dp=new FloatWA(1);
        IntW doh=new IntW(1); FloatWA dsn=new FloatWA(1); IntW sh=new IntW(1);
        vRep.simxGetObjectHandle(clientID,"Proximity_sensor"+sensor,sh,remoteApi.simx_opmode_blocking);
        boolean[] fr={firstSensor0Read,firstSensor1Read,firstSensor2Read,firstSensor3Read,firstSensor4Read,firstSensor5Read};
        int mode=fr[sensor]?remoteApi.simx_opmode_streaming:remoteApi.simx_opmode_buffer;
        vRep.simxReadProximitySensor(clientID,sh.getValue(),det,dp,doh,dsn,mode);
        switch(sensor){case 0:firstSensor0Read=false;break;case 1:firstSensor1Read=false;break;
            case 2:firstSensor2Read=false;break;case 3:firstSensor3Read=false;break;
            case 4:firstSensor4Read=false;break;case 5:firstSensor5Read=false;break;}
        float[] xyz=dp.getArray();
        double d=Math.sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);
        d=Math.round(d*100d)/100d; d=Utils.getDecimal(d,"0.0");
        if(d>=1.0||d==0.0) d=1.0;
        return det.getValue()?d:1.0;
    }
    private double[] readSonars() { for(int i=0;i<getSonarNo();i++) sonarValues[i]=readSonarRange(i); return sonarValues; }
    public double getSonarRange(int s) { return sonarValues[s]; }
    public int    getSonarNo()         { return sonarValues.length; }

    // ==========================================
    //  Camera
    // ==========================================
    private Color[][] readCamera() {
        if(firstCameraRead){vRep.simxGetVisionSensorImage(clientID,cameraHandle.getValue(),resolution,image,2,remoteApi.simx_opmode_streaming);firstCameraRead=false;}
        else{vRep.simxGetVisionSensorImage(clientID,cameraHandle.getValue(),resolution,image,2,remoteApi.simx_opmode_buffer);}
        return imageToColor(image);
    }
    private Color[][] imageToColor(CharWA img) {
        imageArray=img.getArray(); int idx=0;
        for(int i=0;i<resolutionCamera;i++) for(int j=0;j<resolutionCamera;j++){
            Color c=new Color((int)imageArray[idx],(int)imageArray[idx+1],(int)imageArray[idx+2]);
            colorMatrix[i][j]=c; bufferedImage.setRGB(i,j,c.getRGB()); idx+=3;}
        return colorMatrix;
    }
    private int getGrayscale(BufferedImage img,int x,int y){Color c=new Color(img.getRGB(x,y));return(int)(c.getRed()*0.299)+(int)(c.getGreen()*0.587)+(int)(c.getBlue()*0.114);}
    private static BufferedImage rotate(BufferedImage b,double a,boolean col){
        int w=b.getWidth(),h=b.getHeight();
        BufferedImage r=new BufferedImage(w,h,col?b.getType():BufferedImage.TYPE_BYTE_GRAY);
        Graphics2D g=r.createGraphics(); g.rotate(Math.toRadians(a),w/2,h/2); g.drawImage(b,null,0,0); g.dispose(); return r;
    }
    public BufferedImage getImage()          { return rotate(bufferedImage,-90,false); }
    public int           getImageWidth()     { return bufferedImage.getWidth(); }
    public int           getImageHeight()    { return bufferedImage.getHeight(); }
    public int           getImagePixel(int x,int y) { return getGrayscale(bufferedImage,x,y); }
    public void          setImagePixel(int x,int y,int rgb){bufferedImage.setRGB(x,y,rgb+(rgb<<8)+(rgb<<16));}
    public int    getTargetX()        { return point.x(); }
    public int    getTargetY()        { return point.y(); }
    public double getTargetMinScore() { return targetMinScore; }
    public double getTargetMaxScore() { return targetMaxScore; }
    public void   displayImage()      { ImageViewer.display(getImage()); }

    public void templateMatchingCV(BufferedImage image) {
        IplImage src=Java2DFrameUtils.toIplImage(image);
        org.bytedeco.opencv.opencv_core.Mat tm=imread("data/images/marker.jpg",IMREAD_GRAYSCALE);
        IplImage tmp=Java2DFrameUtils.toIplImage(tm);
        IplImage res=cvCreateImage(cvSize(src.width()-tmp.width()+1,src.height()-tmp.height()+1),IPL_DEPTH_32F,1);
        cvMatchTemplate(src,tmp,res,CV_TM_CCOEFF_NORMED);
        DoublePointer minV=new DoublePointer(1),maxV=new DoublePointer(1);
        CvPoint minL=new CvPoint(),maxL=new CvPoint();
        cvMinMaxLoc(res,minV,maxV,minL,maxL,null);
        targetMinScore=minV.get(); targetMaxScore=maxV.get();
        point.x(maxL.x()+tmp.width()); point.y(maxL.y()+tmp.height());
        cvRectangle(src,maxL,point,CvScalar.GRAY,2,8,0);
        ImageViewer.display(Java2DFrameUtils.toBufferedImage(src));
    }

    // ==========================================
    //  Motion
    // ==========================================
    public void forward(){resetButtonsStyle();btnForward.setStyle("-fx-background-color:#7FFF00;");dir='f';}
    public void backward(){resetButtonsStyle();btnBack.setStyle("-fx-background-color:#7FFF00;");dir='b';}
    public void left(){resetButtonsStyle();btnLeft.setStyle("-fx-background-color:#7FFF00;");dir='l';}
    public void right(){resetButtonsStyle();btnRight.setStyle("-fx-background-color:#7FFF00;");dir='r';}
    public void stop(){resetButtonsStyle();btnStop.setStyle("-fx-background-color:#7FFF00;");dir='s';}
    private void resetButtonsStyle(){btnRight.setStyle(defaultButtonStyle);btnStop.setStyle(defaultButtonStyle);btnLeft.setStyle(defaultButtonStyle);btnForward.setStyle(defaultButtonStyle);btnBack.setStyle(defaultButtonStyle);}
    public void setVel(float l,float r){vRep.simxSetJointTargetVelocity(clientID,leftWheelHandle.getValue(),l,remoteApi.simx_opmode_oneshot);vRep.simxSetJointTargetVelocity(clientID,rightWheelHandle.getValue(),r,remoteApi.simx_opmode_oneshot);}
    public void move(float v){setVel(v,v);}
    public void turnSpot(float v){setVel(v,-v);}
    public void turnSharp(float v){if(v>0)setVel(v,0);else setVel(0,-v);}
    public void turnSmooth(float v){if(v>0)setVel(v,v/2);else setVel(-v/2,-v);}
    public void move(float v,int t){motionTimer.setMs(t);motionTimer.restart();while(motionTimer.getState()){move(v);}stop();}
    public void teleoperate(char d,int v){switch(d){case 's':move(0);break;case 'f':move(+v);break;case 'b':move(-v);break;case 'r':turnSpot(+v/2);break;case 'l':turnSpot(-v/2);break;}}

    // ==========================================
    //  Connection
    // ==========================================
    public void connectToVrep() {
        clientID=vRep.simxStart("127.0.0.1",20001,true,true,5000,5);
        if(clientID==-1){btnConnect.setText("Failed");btnConnect.setStyle("-fx-background-color:#FF0000;");vRep.simxFinish(clientID);}
        else{btnConnect.setStyle("-fx-background-color:#7FFF00;");btnConnect.setText("Connected");setup();}
    }
    public void setup() {
        vRep.simxGetObjectHandle(clientID,"JointLeftWheel",leftWheelHandle,remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID,"JointRightWheel",rightWheelHandle,remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID,"Vision_sensor",cameraHandle,remoteApi.simx_opmode_blocking);
        defaultButtonStyle=btnForward.getStyle();
        pw=canvasCamera.getGraphicsContext2D().getPixelWriter();
        ImageViewer.open(resolutionCamera,resolutionCamera,"Camera");
        motionTimer.setSec(1); batteryTimer.setSec(MAX_BATT_TIME);
        motionTimer.start(); batteryTimer.start();
        update.start(); main.start();
    }

    // ==========================================
    //  Sensor Update Thread
    // ==========================================
    private Thread update = new Thread() {
        public void run() {
            while(!simulationComplete) {
                if(runGPS)          gpsValues    = readGPS();
                if(runSensors)      sonarValues  = readSonars();
                if(runWheelEncoder){encoderValues[0]=readLeftWheelEnc();encoderValues[1]=readRightWheelEnc();}
                if(runCamera)       imageCamera  = readCamera();
                if(runMotion)       teleoperate(dir,vel);
                final double[] _g=gpsValues.clone(),_s=sonarValues.clone(),_e=encoderValues.clone();
                final double _b=getBatteryPercentage(); final int _bs=getBatteryTime();
                final double _d=distanceTravelled; final String _bh=activeBehaviour;
                Platform.runLater(()->{
                    if(runGPS){lblGpsX.setText("X:"+_g[0]);lblGpsY.setText("Y:"+_g[1]);lblGpsZ.setText("Z:"+_g[2]);}
                    if(runSensors){lblSensor0.setText(Utils.getDecimal(_s[0],"0.00")+"m");lblSensor1.setText(Utils.getDecimal(_s[1],"0.00")+"m");lblSensor2.setText(Utils.getDecimal(_s[2],"0.00")+"m");lblSensor3.setText(Utils.getDecimal(_s[3],"0.00")+"m");lblSensor4.setText(Utils.getDecimal(_s[4],"0.00")+"m");lblSensor5.setText(Utils.getDecimal(_s[5],"0.00")+"m");}
                    if(runWheelEncoder){lblRightWheel.setText("Right:"+_e[1]);lblLeftWheel.setText("Left:"+_e[0]);}
                    if(lblBattery!=null)lblBattery.setText(String.format("Battery:%.1f%%",_b));
                    if(lblBatteryTime!=null)lblBatteryTime.setText(String.format("Time:%ds",_bs));
                    if(lblBehavior!=null)lblBehavior.setText(_bh);
                    if(lblDistance!=null)lblDistance.setText(String.format("Distance:%.2fm",_d));
                });
                update();
                if(!getBatteryState()){System.err.println("Error: Robot out of battery...");move(0,1000);running=false;break;}
                Delay.ms(1);
            }
        }
    };
    private Thread main=new Thread(){public void run(){while(!simulationComplete){main();Delay.ms(1);}}};

    // =========================================================================
    //                          S T U D E N T   C O D E
    //
    //  Subsumption: AVOID(P1) > TRACK(P2) > CLEAN(P3) > WANDER(P4)
    //  TLU: y = +1 if Σ(w·s) > f,  y = -1 otherwise
    //  Sensors normalised [0,1] before TLU.
    // =========================================================================

    // ── TLU weights ──────────────────────────────────────────────────────────
    private static final double[] AVOID_W  = {0.5,0.6,0.5};
    private static final double   AVOID_F  = 0.15;
    // AVOID_RANGE = 0.25m: fires only when obstacles are within 25 cm.
    // Must be < WALL_MARGIN (0.30m) so the end wall does not fire AVOID
    // before the robot reaches its lane-end waypoint.
    // Geometry: AVOID fires at x = 2.15 - 0.25 = 1.90m, waypoint is at
    // x = 2.15 - 0.30 = 1.85m  →  robot reaches WP at 1.85 before AVOID
    // fires at 1.90. 25cm is still enough to stop before hitting a wall.
    private static final double   AVOID_RANGE = 0.25;

    private static final double[] TRACK_W       = {0.6,0.8};
    private static final double   TRACK_F       = 1.0;
    private static final double   TRACK_BATT_PCT= 20.0;
    private static final double   TRACK_CAM_SCORE=0.55;

    private static final double[] CLEAN_W       = {1.0};
    private static final double   CLEAN_F       = 0.5;
    private static final double   CLEAN_BATT_PCT= 40.0;

    private static final double[] WANDER_W = {1.0};
    private static final double   WANDER_F = 0.0;

    // ── Robot dimensions ──────────────────────────────────────────────────────
    private static final float  DRIVE_SPEED  = 3.0f;
    private static final float  TURN_SPEED   = 2.0f;
    private static final double WHEEL_RADIUS = 0.072;
    private static final double WHEEL_BASE   = 0.331;

    // ── Room & waypoints ──────────────────────────────────────────────────────
    //  Hard constraint:  WALL_MARGIN > AVOID_RANGE > WP_REACH_RADIUS
    //  Current values:   0.30        > 0.25        > 0.22          ✓
    //
    //  WALL_MARGIN = 0.30m → waypoints at ±1.85m (30cm from ±2.15 walls).
    //  Robot stops cleaning 30cm from the wall — close enough to look good,
    //  far enough that the front sonar (AVOID_RANGE=0.25m) never fires for
    //  the end wall before the waypoint is registered.
    //
    //  Why both must change together:
    //    AVOID fires when front sensor reads < AVOID_RANGE.
    //    End wall is at 2.15m, so AVOID fires when robot is at x > 1.90m.
    //    Lane-end waypoint is at x = 1.85m.  1.85 < 1.90  →  robot reaches
    //    waypoint before AVOID fires  ✓
    private static final double WALL_MARGIN = 0.30;
    private static final double CLEAN_MIN_X = -2.15 + WALL_MARGIN;   // -1.85
    private static final double CLEAN_MAX_X =  2.15 - WALL_MARGIN;   // +1.85
    private static final double CLEAN_MIN_Y = -2.15 + WALL_MARGIN;   // -1.85
    private static final double CLEAN_MAX_Y =  2.15 - WALL_MARGIN;   // +1.85
    private static final double LANE_SPACING = 0.35;

    private static final double WP_REACH_RADIUS       = 0.22;                     // metres — < LANE_SPACING/2 to prevent skipping
    private static final double HEADING_TURN_THRESHOLD = Math.toRadians(25.0);    // radians

    // =========================================================================
    //  HYBRID HEADING ESTIMATOR
    //
    //  PRIMARY  — Encoder differential integration:
    //    dθ = (dR - dL) / WHEEL_BASE
    //    dL, dR = arc-length deltas from encoder revolution deltas.
    //    Updates every tick, works during spinning AND translating.
    //    Sign: dR > dL → turned left → heading increases (CCW positive, 0 = +X).
    //
    //  SECONDARY — GPS calibration:
    //    When GPS has moved ≥ GPS_CALIB_DIST since last calibration point,
    //    snap headingEst to the GPS movement bearing.
    //    Eliminates accumulated encoder drift.
    //
    //  Why deadlock is impossible:
    //    During spinning, encoder integration changes headingEst each tick.
    //    After rotating ~headingError radians the error drops below
    //    HEADING_TURN_THRESHOLD and the robot starts translating forward.
    //    First forward movement triggers GPS calibration → drift cleared.
    // =========================================================================
    private static final double GPS_CALIB_DIST = 0.05;   // metres before GPS recalibrates heading

    private double  headingEst   = 0.0;     // current heading estimate (rad, 0 = facing +X)

    // Encoder state
    private double  encHL_prev  = 0.0, encHR_prev = 0.0;
    private boolean encHInit    = false;

    // GPS calibration state
    private double  gpsCalibX   = Double.NaN;
    private double  gpsCalibY   = Double.NaN;

    private double normAngle(double a) {
        while(a> Math.PI) a-=2*Math.PI;
        while(a<-Math.PI) a+=2*Math.PI;
        return a;
    }

    /**
     * Update heading every tick.
     * Step 1: integrate encoder differential (works while spinning).
     * Step 2: calibrate to GPS bearing whenever robot has moved ≥ GPS_CALIB_DIST.
     */
    private void updateHeading(double gpsX, double gpsY) {
        // ── Encoder integration ───────────────────────────────────────────
        double encL = getLeftWheelEnc();
        double encR = getRightWheelEnc();
        if (!encHInit) {
            encHL_prev = encL; encHR_prev = encR; encHInit = true;
        } else {
            double dL = (encL - encHL_prev) * 2.0 * Math.PI * WHEEL_RADIUS;
            double dR = (encR - encHR_prev) * 2.0 * Math.PI * WHEEL_RADIUS;
            // dR - dL > 0 → right wheel went further → turned left → heading increases
            headingEst = normAngle(headingEst + (dR - dL) / WHEEL_BASE);
        }
        encHL_prev = encL; encHR_prev = encR;

        // ── GPS calibration ───────────────────────────────────────────────
        if (Double.isNaN(gpsCalibX)) { gpsCalibX = gpsX; gpsCalibY = gpsY; return; }
        double moved = Math.hypot(gpsX - gpsCalibX, gpsY - gpsCalibY);
        if (moved >= GPS_CALIB_DIST) {
            // Snap heading to GPS movement bearing → removes encoder drift
            headingEst = Math.atan2(gpsY - gpsCalibY, gpsX - gpsCalibX);
            gpsCalibX = gpsX; gpsCalibY = gpsY;
        }
    }

    // =========================================================================
    //  GPS WAYPOINT BOUSTROPHEDON PATH
    //  Waypoints generated once. Navigated sequentially.
    //  Lane structure:
    //    Lane 0: y=CLEAN_MIN_Y, drive West→East
    //    Lane 1: y+=SPACING,    drive East→West
    //    Lane 2: y+=SPACING,    drive West→East  ...
    //  Each lane = 2 waypoints: [lane-start, lane-end].
    // =========================================================================
    private List<double[]> waypoints      = new ArrayList<>();
    private int            wpIndex        = 0;
    private boolean        cleanStarted   = false;
    private int            lanesCompleted = 0;   // lanes in current pass
    private int            cleanPassCount = 0;   // how many full passes completed
    private int            totalLanes     = 0;   // cumulative across all passes
    private int            wpSkipped      = 0;   // total waypoints skipped due to obstacles

    // ── Waypoint skip timeout ─────────────────────────────────────────────────
    // If the robot has been trying to reach the same waypoint for longer than
    // WP_TIMEOUT_MS, the waypoint is assumed blocked by an obstacle → skip it.
    private static final long WP_TIMEOUT_MS = 6_000;   // 6 seconds per waypoint
    private long   wpStartTime    = 0;    // wall-clock ms when current WP targeting began
    private int    wpLastIndex    = -1;   // which WP the timer is currently tracking

    // ── Stuck detector ────────────────────────────────────────────────────────
    // If GPS has moved less than STUCK_DIST_M in STUCK_CHECK_MS, the robot is
    // physically stuck (wedged against obstacle). Execute a backup escape maneuver.
    private static final double STUCK_DIST_M    = 0.08;   // metres — less than this = stuck
    private static final long   STUCK_CHECK_MS  = 3_000;  // check window in ms
    private static final long   BACKUP_DURATION_MS = 1_200; // how long to reverse
    private long   stuckCheckStart   = 0;
    private double stuckCheckX       = Double.NaN;
    private double stuckCheckY       = Double.NaN;
    private boolean inBackup         = false;
    private long    backupStart      = 0;
    private int     backupDir        = 1;  // +1 or -1, alternates each escape

    /**
     * Committed turn direction for navigateToWaypoint().
     *  0  = not in spin mode (direction will be set on next spin entry)
     * -1  = spinning left  (headingEst increasing, right wheel forward)
     * +1  = spinning right (headingEst decreasing, left wheel forward)
     *
     * Set ONCE on entry to spin mode, held until |err| drops below threshold.
     * Without this, normAngle() flips sign the instant headingEst crosses ±π
     * during a 180° U-turn, reversing the spin every tick → robot oscillates.
     */
    private int committedTurnDir = 0;

    private void generateWaypoints() {
        waypoints.clear();
        boolean ltr = true;
        int lane = 0;
        for (double y = CLEAN_MIN_Y; y <= CLEAN_MAX_Y + 0.001; y += LANE_SPACING) {
            double cy = Math.min(y, CLEAN_MAX_Y);
            if (ltr) { waypoints.add(new double[]{CLEAN_MIN_X,cy}); waypoints.add(new double[]{CLEAN_MAX_X,cy}); }
            else      { waypoints.add(new double[]{CLEAN_MAX_X,cy}); waypoints.add(new double[]{CLEAN_MIN_X,cy}); }
            ltr = !ltr; lane++;
        }
        System.out.printf("[CLEAN] %d waypoints, %d lanes%n", waypoints.size(), lane);
        for (int i=0;i<waypoints.size();i++)
            System.out.printf("  WP%02d: (%.2f, %.2f)%n",i,waypoints.get(i)[0],waypoints.get(i)[1]);
    }

    /**
     * Steer toward (tx,ty) from current GPS (gpsX,gpsY).
     *
     * Bearing error  = atan2(ty-gpsY, tx-gpsX) - headingEst
     * If |error| > 25°: spin toward bearing (encoder integration moves headingEst → never stuck).
     * If |error| ≤ 25°: drive forward + proportional lateral correction.
     *
     * Convention check (critical for correct turn direction):
     *   setVel(leftVel, rightVel)
     *   Positive L, zero R  → turns right  → heading decreases
     *   Zero L, positive R  → turns left   → heading increases
     *   So: dθ = (dR - dL) / WHEEL_BASE  [CCW positive] ✓
     *
     *   For heading error > 0 (waypoint is to our left, CCW):
     *     Need to turn left → increase heading → dR > dL
     *     → setVel(−TURN, +TURN)  which is setVel(spin, −spin) with spin=−TURN ✓
     */
    private void navigateToWaypoint(double gpsX, double gpsY, double tx, double ty) {
        double dist    = Math.hypot(tx-gpsX, ty-gpsY);
        double bearing = Math.atan2(ty-gpsY, tx-gpsX);
        double err     = normAngle(bearing - headingEst);

        // Slow down near waypoint to avoid overshoot
        float speed = (dist < 0.50) ? DRIVE_SPEED * 0.5f : DRIVE_SPEED;

        if (Math.abs(err) > HEADING_TURN_THRESHOLD) {
            // ── SPIN MODE ────────────────────────────────────────────────
            // Commit to a turn direction on the FIRST tick of a spin.
            // Do NOT re-evaluate err sign while spinning — normAngle() wraps
            // at ±π so a 180° U-turn causes err to flip sign every tick if
            // we recalculate, making the robot oscillate left↔right forever.
            if (committedTurnDir == 0) {
                // err > 0 → waypoint is to our left → turn left → dir = -1
                // err < 0 → waypoint is to our right → turn right → dir = +1
                committedTurnDir = (err >= 0) ? -1 : +1;
            }
            // committedTurnDir = -1: left turn → right wheel fwd, left bwd
            //   setVel(spin, -spin) with spin < 0 → setVel(-TURN, +TURN) ✓
            // committedTurnDir = +1: right turn → left wheel fwd, right bwd
            //   setVel(spin, -spin) with spin > 0 → setVel(+TURN, -TURN) ✓
            float spin = committedTurnDir * TURN_SPEED;   // -1*T or +1*T
            setVel(spin, -spin);

        } else {
            // ── DRIVE MODE ───────────────────────────────────────────────
            // Turn complete — reset so next turn gets a fresh direction check.
            committedTurnDir = 0;
            // Proportional steering correction.
            // err > 0 → drifted right → steer left → slow left, speed right
            double Kp = 2.5;
            double steer = Math.max(-speed, Math.min(speed, Kp * err));
            setVel((float)(speed - steer), (float)(speed + steer));
        }
    }

    // CLEAN phase label
    private enum CleanPhase { WAITING_GPS, NAVIGATING, COMPLETE }
    private CleanPhase cleanPhase = CleanPhase.WAITING_GPS;

    // ── Wander state ──────────────────────────────────────────────────────────
    private static final int WANDER_INTERVAL = 400;
    private static final int WANDER_DURATION = 200;
    private int    wanderTick  = 0;
    private double wanderSteer = 0.0;

    // ── Track/dock state ──────────────────────────────────────────────────────
    private boolean docked            = false;
    private boolean chargingDone      = false;
    private boolean simulationComplete = false;   // set true to stop both threads
    private long    chargeStart       = 0;
    private static final long CHARGE_DURATION_MS = 10_000;   // 10 s simulated charge

    // ── Metrics ───────────────────────────────────────────────────────────────
    private double distanceTravelled=0, prevEncLeft=0, prevEncRight=0;
    private boolean metricsInit=false;
    private List<String> dataLog=new ArrayList<>(), eventLog=new ArrayList<>();
    private boolean  dataExported=false;
    private int      loopCount=0;
    private static final int LOG_INTERVAL = 50;   // log every 50 ticks → ~denser GPS trail for scatter plot
    private int    avoidCount=0,trackCount=0,cleanCount=0,wanderCount=0;
    private String activeBehaviour="INIT", prevBehaviour="INIT";
    private int    behavTransitions=0;
    private boolean loggedBatt40=false, loggedBatt20=false;
    private boolean autonomousMode=false, initialized=false;

    // =========================================================================
    //  TLU
    // =========================================================================
    private boolean tlu(double[] w, double[] s, double f) {
        double sum=0; for(int i=0;i<w.length&&i<s.length;i++) sum+=w[i]*s[i]; return sum>f;
    }

    private void logEvent(String type, String detail) {
        double t=getBatteryTime();
        eventLog.add(String.format("%.2f,%d,%s,%s",t,loopCount,type,detail));
        System.out.printf("[EVENT|t=%.1fs] %-22s %s%n",t,type,detail);
    }

    private void updateDistance() {
        double eL=getLeftWheelEnc(),eR=getRightWheelEnc();
        if(!metricsInit){prevEncLeft=eL;prevEncRight=eR;metricsInit=true;return;}
        double cL=eL*2*Math.PI*WHEEL_RADIUS, cR=eR*2*Math.PI*WHEEL_RADIUS;
        double pL=prevEncLeft*2*Math.PI*WHEEL_RADIUS, pR=prevEncRight*2*Math.PI*WHEEL_RADIUS;
        distanceTravelled+=Utils.getEuclidean(pL,pR,cL,cR);
        prevEncLeft=eL; prevEncRight=eR;
    }

    // =========================================================================
    //  BEHAVIOUR 1 — AVOID
    //  TLU inputs: normalised proximity from US0, US1, US2 (front sensors)
    //  proximity = max(0, (AVOID_RANGE - range) / AVOID_RANGE)
    // =========================================================================
    private boolean shouldAvoid(double s0,double s1,double s2) {
        double p0=Math.max(0,(AVOID_RANGE-s0)/AVOID_RANGE);
        double p1=Math.max(0,(AVOID_RANGE-s1)/AVOID_RANGE);
        double p2=Math.max(0,(AVOID_RANGE-s2)/AVOID_RANGE);
        return tlu(AVOID_W,new double[]{p0,p1,p2},AVOID_F);
    }
    private void executeAvoid(double s0,double s1,double s2) {
        avoidCount++; activeBehaviour="AVOID";
        double p0=Math.max(0,(AVOID_RANGE-s0)/AVOID_RANGE);
        double p1=Math.max(0,(AVOID_RANGE-s1)/AVOID_RANGE);
        double p2=Math.max(0,(AVOID_RANGE-s2)/AVOID_RANGE);
        if(s1<0.15||s0<0.12||s2<0.12){double st=(p0>p2)?-1.5:1.5;setVel((float)(-1.0+st),(float)(-1.0-st));return;}
        float speed; double steer;
        if(p1>0.6){speed=1.5f;steer=(p0>p2)?-2.5:2.5;}
        else{speed=2.5f;steer=(p0-p2)*2.5+p1*1.5;steer=Math.max(-3,Math.min(3,steer));}
        setVel((float)(speed+steer),(float)(speed-steer));
    }

    // =========================================================================
    //  BEHAVIOUR 2 — TRACK
    //  TLU input: battery ≤ 20%  (camera NOT required — fixes the main nav problem)
    //
    //  Navigation strategy:
    //    Phase A (dist > 0.60m): GPS-direct bearing navigation using the same
    //      hybrid heading estimator as CLEAN. Robot drives straight to charger
    //      GPS coordinates. No camera needed → always fires when batt ≤ 20%.
    //    Phase B (dist ≤ 0.60m): Camera visual servoing for precise final dock.
    //      If camera still can't see marker, continue GPS approach at slow speed.
    //    Phase C (dist < 0.30m): Docked. Stop, charge, end simulation.
    //
    //  Why the old approach failed:
    //    Old TLU: lowBatt AND camScore — if camera can't see marker, TRACK never
    //    fires, robot wanders for minutes burning the last 10% of battery.
    //    New TLU: lowBatt alone — always navigates toward charger GPS when low.
    // =========================================================================
    private boolean shouldTrack(double batt, double cam) {
        // Fire on battery alone — camera not required to start homing
        double lowBatt = (batt <= TRACK_BATT_PCT) ? 1.0 : 0.0;
        return tlu(new double[]{1.0}, new double[]{lowBatt}, 0.5);
    }

    private void executeTrack(double gpsX, double gpsY, double cam) {
        trackCount++; activeBehaviour = "TRACK";
        double dist = Math.hypot(gpsX - CHARGER_XCOORD, gpsY - CHARGER_YCOORD);

        // ── Phase C: Docked ───────────────────────────────────────────────
        if (dist < 0.30) {
            setVel(0, 0); docked = true;
            if (!chargingDone) {
                if (chargeStart == 0) {
                    chargeStart = System.currentTimeMillis();
                    System.out.println("============================================");
                    System.out.printf("  DOCKED at GPS(%.2f,%.2f)%n", gpsX, gpsY);
                    System.out.printf("  Charging — battery will rise to 100%%%n");
                    System.out.println("============================================");
                    logEvent("DOCKED", String.format("pos=(%.2f,%.2f) batt=%.1f%%", gpsX, gpsY, getBatteryPercentage()));
                }
                // Grow chargeOffset so getBatteryCapacity() increases each tick
                double elapsedChargeSec = (System.currentTimeMillis() - chargeStart) / 1000.0;
                chargeOffset = elapsedChargeSec * CHARGE_RATE_PER_SEC;

                // Print charging progress every ~2s
                if ((int)(elapsedChargeSec) % 2 == 0 && (int)(elapsedChargeSec) > 0) {
                    System.out.printf("  Charging: %.0f%%%n", getBatteryPercentage());
                }

                if (getBatteryPercentage() >= 99.0) {
                    chargingDone = true;
                    System.out.println("============================================");
                    System.out.println("  CHARGING COMPLETE — simulation ending");
                    System.out.printf("  Battery: %.1f%%  Distance: %.2fm%n", getBatteryPercentage(), distanceTravelled);
                    System.out.printf("  Clean passes: %d  Total lanes: %d%n", cleanPassCount, totalLanes);
                    System.out.println("============================================");
                    logEvent("CHARGE_COMPLETE", String.format("pos=(%.2f,%.2f) dist=%.2fm passes=%d batt=%.1f%%",
                            gpsX, gpsY, distanceTravelled, cleanPassCount, getBatteryPercentage()));
                    exportData("CHARGE_COMPLETE");
                    simulationComplete = true;
                }
            }
            return;
        }

        // ── Phase B: Close — camera fine approach ─────────────────────────
        if (dist <= 0.60 && cam >= TRACK_CAM_SCORE) {
            double normX = Math.max(-1, Math.min(1, (getTargetX() - getImageWidth() / 2.0) / (getImageWidth() / 2.0)));
            float speed = (dist < 0.40) ? DRIVE_SPEED * 0.3f : DRIVE_SPEED * 0.5f;
            double steer = Math.max(-speed, Math.min(speed, 1.8 * normX));
            setVel((float)(speed + steer), (float)(speed - steer));
            return;
        }

        // ── Phase A: GPS-direct bearing navigation ────────────────────────
        // Same approach as CLEAN waypoint navigation — compute bearing to
        // charger GPS coords, use hybrid heading estimator, proportional steer.
        updateHeading(gpsX, gpsY);
        double bearing = Math.atan2(CHARGER_YCOORD - gpsY, CHARGER_XCOORD - gpsX);
        double err = normAngle(bearing - headingEst);
        float speed = (dist < 0.80) ? DRIVE_SPEED * 0.5f : DRIVE_SPEED;

        if (Math.abs(err) > HEADING_TURN_THRESHOLD) {
            if (committedTurnDir == 0) committedTurnDir = (err >= 0) ? -1 : +1;
            setVel(committedTurnDir * TURN_SPEED, -committedTurnDir * TURN_SPEED);
        } else {
            committedTurnDir = 0;
            double steer = Math.max(-speed, Math.min(speed, 2.5 * err));
            setVel((float)(speed - steer), (float)(speed + steer));
        }
    }

    // =========================================================================
    //  BEHAVIOUR 3 — CLEAN
    //  TLU input: battery ≥ 40% (1.0) else 0.0
    //
    //  GPS Waypoint Navigation with Hybrid Heading:
    //   • GPS validity guard: holds still if GPS is bogus (cold-start overflow).
    //   • Generates waypoints once when GPS becomes valid.
    //   • Calls updateHeading() every tick (encoder-primary, GPS-calibrated).
    //   • Reaches waypoint → advance index. Spin+drive toward next.
    // =========================================================================
    private boolean shouldClean(double batt) {
        return tlu(CLEAN_W,new double[]{batt>=CLEAN_BATT_PCT?1.0:0.0},CLEAN_F);
    }

    private void executeClean(double gpsX, double gpsY) {
        cleanCount++; activeBehaviour="CLEAN";

        // ── GPS validity guard ────────────────────────────────────────────
        // Do NOT start until GPS reports a sane room coordinate.
        // This prevents the Long.MAX_VALUE cold-start overflow from
        // corrupting the waypoint list or heading calibration.
        if (!isGpsValid(gpsX, gpsY)) {
            cleanPhase = CleanPhase.WAITING_GPS;
            setVel(0, 0);
            return;
        }

        // ── First-time initialisation ─────────────────────────────────────
        if (!cleanStarted) {
            cleanStarted   = true;
            cleanPhase     = CleanPhase.NAVIGATING;
            wpIndex        = 0;
            lanesCompleted = 0;
            generateWaypoints();
            // Seed GPS calibration reference at confirmed valid position
            gpsCalibX = gpsX;
            gpsCalibY = gpsY;
            logEvent("CLEAN_START",
                    String.format("pos=(%.2f,%.2f) totalWP=%d", gpsX, gpsY, waypoints.size()));
        }

        // ── All waypoints complete — restart the cleaning pass ────────────
        //
        // Assignment requires the robot to operate continuously:
        //   battery 40-100%  →  CLEAN (this behaviour)
        //   battery 20-40%   →  WANDER (CLEAN and TRACK both inactive → fallback)
        //   battery ≤ 20%    →  TRACK  (return to charging station)
        //
        // So when all waypoints are done we do NOT stop.  Instead we reset the
        // clean state so the subsumption re-enters CLEAN on the next tick and
        // starts another pass from the beginning.  If the battery has dropped
        // below CLEAN_BATT_PCT by then, CLEAN won't fire and WANDER/TRACK
        // will run instead — exactly the intended behaviour.
        if (wpIndex >= waypoints.size()) {
            cleanPassCount++;
            totalLanes += lanesCompleted;
            logEvent("CLEAN_PASS_DONE",
                    String.format("pass=%d lanes=%d totalLanes=%d dist=%.2fm pos=(%.2f,%.2f)",
                            cleanPassCount, lanesCompleted, totalLanes,
                            distanceTravelled, gpsX, gpsY));
            System.out.printf("[CLEAN] Pass %d complete — %d lanes, %.2fm total distance%n",
                    cleanPassCount, lanesCompleted, distanceTravelled);
            // Reset clean state → next CLEAN tick re-initialises from current GPS
            cleanStarted   = false;
            cleanPhase     = CleanPhase.WAITING_GPS;
            wpIndex        = 0;
            lanesCompleted = 0;
            // Reset heading estimator so new pass gets a fresh calibration
            encHInit         = false;
            gpsCalibX        = Double.NaN;
            gpsCalibY        = Double.NaN;
            committedTurnDir = 0;
            // Reset obstacle-handling state
            wpStartTime      = 0;
            wpLastIndex      = -1;
            stuckCheckStart  = 0;
            stuckCheckX      = Double.NaN;
            stuckCheckY      = Double.NaN;
            inBackup         = false;
            return;
        }

        // ── Update heading (encoder+GPS) ──────────────────────────────────
        updateHeading(gpsX, gpsY);

        // ── Check waypoint reached ────────────────────────────────────────
        double[] wp   = waypoints.get(wpIndex);
        double   dist = Math.hypot(wp[0]-gpsX, wp[1]-gpsY);

        if (dist < WP_REACH_RADIUS) {
            if (wpIndex % 2 == 1) {
                lanesCompleted++;
                logEvent("LANE_DONE", String.format("lane=%d pos=(%.2f,%.2f)", lanesCompleted, gpsX, gpsY));
            }
            logEvent("WP_REACHED", String.format("wp=%d pos=(%.2f,%.2f) err=%.3fm", wpIndex, gpsX, gpsY, dist));
            wpIndex++;
            committedTurnDir = 0;
            wpStartTime      = 0;    // reset timeout for next WP
            wpLastIndex      = -1;
            stuckCheckStart  = 0;    // reset stuck detector
            stuckCheckX      = Double.NaN;
            stuckCheckY      = Double.NaN;
            inBackup         = false;
            return;
        }

        long now = System.currentTimeMillis();

        // ── MECHANISM 1: Backup escape maneuver ───────────────────────────
        // If currently executing a backup, keep reversing until timer expires,
        // then release and let normal navigation resume. AVOID stays active above
        // us in the subsumption stack, so this is only reached when clear.
        if (inBackup) {
            long elapsed = now - backupStart;
            if (elapsed < BACKUP_DURATION_MS) {
                // Reverse + slight turn to escape the pocket
                float rev = -DRIVE_SPEED * 0.6f;
                float turn = backupDir * TURN_SPEED * 0.4f;
                setVel(rev - turn, rev + turn);
                return;
            } else {
                // Backup complete — release, reset stuck detector, continue
                inBackup        = false;
                backupDir       = -backupDir;   // alternate direction each escape
                stuckCheckStart = 0;
                stuckCheckX     = Double.NaN;
                stuckCheckY     = Double.NaN;
                committedTurnDir= 0;
                logEvent("BACKUP_DONE", String.format("wp=%d pos=(%.2f,%.2f)", wpIndex, gpsX, gpsY));
            }
        }

        // ── MECHANISM 2: Stuck detector ───────────────────────────────────
        // Seed the reference position on first call for this check window.
        if (stuckCheckStart == 0 || Double.isNaN(stuckCheckX)) {
            stuckCheckStart = now;
            stuckCheckX     = gpsX;
            stuckCheckY     = gpsY;
        } else if (now - stuckCheckStart >= STUCK_CHECK_MS) {
            double moved = Math.hypot(gpsX - stuckCheckX, gpsY - stuckCheckY);
            if (moved < STUCK_DIST_M) {
                // Robot has barely moved in STUCK_CHECK_MS → physically trapped
                inBackup    = true;
                backupStart = now;
                logEvent("STUCK_ESCAPE", String.format(
                    "wp=%d moved=%.3fm pos=(%.2f,%.2f) dir=%d",
                    wpIndex, moved, gpsX, gpsY, backupDir));
                System.out.printf("[STUCK] WP%d blocked — backing up (dir=%d)%n", wpIndex, backupDir);
                return;
            }
            // Moved enough — reset window for next check
            stuckCheckStart = now;
            stuckCheckX     = gpsX;
            stuckCheckY     = gpsY;
        }

        // ── MECHANISM 3: Waypoint skip timeout ───────────────────────────
        // If the robot has been targeting this waypoint longer than WP_TIMEOUT_MS
        // without reaching it, the waypoint is permanently blocked → skip it.
        if (wpLastIndex != wpIndex) {
            // New waypoint — start fresh timer
            wpLastIndex = wpIndex;
            wpStartTime = now;
        } else if (now - wpStartTime > WP_TIMEOUT_MS) {
            wpSkipped++;
            logEvent("WP_SKIPPED", String.format(
                "wp=%d pos=(%.2f,%.2f) dist=%.2fm timeout=%ds totalSkipped=%d",
                wpIndex, gpsX, gpsY, dist, WP_TIMEOUT_MS/1000, wpSkipped));
            System.out.printf("[CLEAN] WP%d skipped (obstacle) — %.1fs elapsed, dist=%.2fm%n",
                wpIndex, (now - wpStartTime)/1000.0, dist);
            wpIndex++;
            committedTurnDir = 0;
            wpStartTime      = 0;
            wpLastIndex      = -1;
            stuckCheckStart  = 0;
            stuckCheckX      = Double.NaN;
            stuckCheckY      = Double.NaN;
            inBackup         = false;
            return;
        }

        // ── Steer toward waypoint ─────────────────────────────────────────
        navigateToWaypoint(gpsX, gpsY, wp[0], wp[1]);
    }

    // =========================================================================
    //  BEHAVIOUR 4 — WANDER
    //  TLU input: battery > 0% → always fires as fallback
    // =========================================================================
    private boolean shouldWander(double batt) {
        return tlu(WANDER_W,new double[]{batt>0?1.0:0.0},WANDER_F);
    }
    private void executeWander() {
        wanderCount++; activeBehaviour="WANDER";
        if(cleanStarted){
            cleanStarted=false; cleanPhase=CleanPhase.WAITING_GPS;
            wpIndex=0; encHInit=false; gpsCalibX=Double.NaN; gpsCalibY=Double.NaN;
            committedTurnDir=0;
            wpStartTime=0; wpLastIndex=-1;
            stuckCheckStart=0; stuckCheckX=Double.NaN; stuckCheckY=Double.NaN;
            inBackup=false;
        }
        wanderTick++;
        if(wanderTick%(WANDER_INTERVAL+WANDER_DURATION)==WANDER_INTERVAL){wanderSteer=(Math.random()*2-1)*1.2;logEvent("WANDER_TURN",String.format("steer=%.2f",wanderSteer));}
        if(wanderTick%(WANDER_INTERVAL+WANDER_DURATION)==0) wanderSteer=0.0;
        setVel((float)(DRIVE_SPEED+wanderSteer),(float)(DRIVE_SPEED-wanderSteer));
    }

    // =========================================================================
    //  DATA EXPORT
    //  Files are written to the Java working directory (project root).
    //  Three output files per run, timestamped to avoid overwriting:
    //    trajectory_<ts>.csv  — full sensor log every 50 ticks (for all graphs)
    //    gps_scatter_<ts>.csv — GPS x,y only, 1 row/tick  (for GPS scatter plot)
    //    events_<ts>.csv      — behaviour/waypoint events  (for timeline analysis)
    //    report_<ts>.txt      — summary statistics
    // =========================================================================
    private void exportData(String reason) {
        // Do NOT guard with dataExported — always write final state including
        // the full charging sequence data.
        dataExported = true;
        double elapsed = getBatteryTime();
        long   ts      = System.currentTimeMillis();
        String prefix  = "run_" + ts;

        // ── Full trajectory CSV ───────────────────────────────────────────
        String tf = prefix + "_trajectory.csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(tf))) {
            pw.println("time_sec,gps_x,gps_y,battery_pct,left_enc,right_enc," +
                       "distance_m,behaviour,sonar_0,sonar_1,sonar_2,cam_score," +
                       "clean_phase,wp_index");
            for (String r : dataLog) pw.println(r);
            System.out.println("[EXPORT] Trajectory: " + tf + " (" + dataLog.size() + " rows)");
        } catch (Exception e) { System.err.println("[EXPORT] trajectory: " + e.getMessage()); }

        // ── GPS scatter plot CSV (x,y + behaviour label for colour coding) ─
        String gf = prefix + "_gps_scatter.csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(gf))) {
            pw.println("gps_x,gps_y,behaviour,battery_pct,time_sec");
            for (String r : dataLog) {
                String[] c = r.split(",");
                if (c.length >= 8)
                    pw.printf("%s,%s,%s,%s,%s%n", c[1], c[2], c[7], c[3], c[0]);
            }
            System.out.println("[EXPORT] GPS scatter: " + gf);
        } catch (Exception e) { System.err.println("[EXPORT] gps_scatter: " + e.getMessage()); }

        // ── Distance-time CSV (for distance vs time correlation plot) ──────
        String df = prefix + "_distance_time.csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(df))) {
            pw.println("time_sec,distance_m,battery_pct,behaviour");
            for (String r : dataLog) {
                String[] c = r.split(",");
                if (c.length >= 8)
                    pw.printf("%s,%s,%s,%s%n", c[0], c[6], c[3], c[7]);
            }
            System.out.println("[EXPORT] Distance-time: " + df);
        } catch (Exception e) { System.err.println("[EXPORT] distance_time: " + e.getMessage()); }

        // ── Events CSV ───────────────────────────────────────────────────
        String ef = prefix + "_events.csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(ef))) {
            pw.println("time_sec,tick,event_type,detail");
            for (String r : eventLog) pw.println(r);
            System.out.println("[EXPORT] Events: " + ef + " (" + eventLog.size() + " events)");
        } catch (Exception e) { System.err.println("[EXPORT] events: " + e.getMessage()); }

        // ── Behaviour statistics CSV (for pie/bar chart) ──────────────────
        int tot = avoidCount + trackCount + cleanCount + wanderCount;
        String bf = prefix + "_behaviour_stats.csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(bf))) {
            pw.println("behaviour,ticks,percentage");
            pw.printf("AVOID,%d,%.2f%n",  avoidCount,  100.0*avoidCount /Math.max(1,tot));
            pw.printf("TRACK,%d,%.2f%n",  trackCount,  100.0*trackCount /Math.max(1,tot));
            pw.printf("CLEAN,%d,%.2f%n",  cleanCount,  100.0*cleanCount /Math.max(1,tot));
            pw.printf("WANDER,%d,%.2f%n", wanderCount, 100.0*wanderCount/Math.max(1,tot));
            System.out.println("[EXPORT] Behaviour stats: " + bf);
        } catch (Exception e) { System.err.println("[EXPORT] behaviour_stats: " + e.getMessage()); }

        // ── Summary report TXT ────────────────────────────────────────────
        String rf = prefix + "_report.txt";
        try (PrintWriter pw = new PrintWriter(new FileWriter(rf))) {
            pw.println("=== EXPERIMENT REPORT ===");
            pw.printf("End reason         : %s%n", reason);
            pw.printf("Total time         : %.1f s (%.1f min)%n", elapsed, elapsed/60.0);
            pw.printf("Distance travelled : %.2f m%n", distanceTravelled);
            pw.printf("Final battery      : %.1f%%%n", getBatteryPercentage());
            pw.printf("Final GPS          : (%.2f, %.2f)%n", getGPSX(), getGPSY());
            pw.println();
            pw.printf("Clean passes       : %d%n", cleanPassCount);
            pw.printf("Lanes this pass    : %d%n", lanesCompleted);
            pw.printf("Total lanes all    : %d%n", totalLanes);
            pw.printf("WP reached         : %d / %d%n", wpIndex, waypoints.size());
            pw.printf("WP skipped         : %d (blocked by obstacles)%n", wpSkipped);
            pw.printf("Docked             : %b%n", docked);
            pw.printf("Behaviour changes  : %d%n", behavTransitions);
            pw.println();
            pw.println("Behaviour ticks:");
            pw.printf("  AVOID  : %6d  (%.1f%%)%n", avoidCount,  100.0*avoidCount /Math.max(1,tot));
            pw.printf("  TRACK  : %6d  (%.1f%%)%n", trackCount,  100.0*trackCount /Math.max(1,tot));
            pw.printf("  CLEAN  : %6d  (%.1f%%)%n", cleanCount,  100.0*cleanCount /Math.max(1,tot));
            pw.printf("  WANDER : %6d  (%.1f%%)%n", wanderCount, 100.0*wanderCount/Math.max(1,tot));
            System.out.println("[EXPORT] Report: " + rf);
        } catch (Exception e) { System.err.println("[EXPORT] report: " + e.getMessage()); }

        System.out.println("============================================");
        System.out.printf("  ALL FILES SAVED with prefix: %s%n", prefix);
        System.out.printf("  Files written to: %s%n", System.getProperty("user.dir"));
        System.out.println("============================================");
    }

    public void update() { templateMatchingCV(getImage()); }

    public void toggleAutonomousMode() {
        autonomousMode=!autonomousMode;
        if(autonomousMode){runMotion=false;btnAutoMode.setText("Auto Mode: ON");btnAutoMode.setStyle("-fx-background-color:#7FFF00;");logEvent("AUTO_MODE","ENABLED");}
        else{runMotion=true;setVel(0,0);btnAutoMode.setText("Auto Mode: OFF");btnAutoMode.setStyle("");logEvent("AUTO_MODE","DISABLED");}
    }

    // =========================================================================
    //  main() — runs every tick
    // =========================================================================
    public void main() {
        if(!initialized){
            initialized=true;
            int lanes=(int)Math.ceil((CLEAN_MAX_Y-CLEAN_MIN_Y)/LANE_SPACING)+1;
            System.out.println("============================================");
            System.out.printf("  ROBOT READY — autonomous mode OFF%n");
            System.out.printf("  Room [%.2f..%.2f] x [%.2f..%.2f]  Lanes:%d  LaneSpacing:%.2fm%n",
                CLEAN_MIN_X,CLEAN_MAX_X,CLEAN_MIN_Y,CLEAN_MAX_Y,lanes,LANE_SPACING);
            System.out.printf("  WPReach:%.2fm  TurnThreshold:%.0f°  GPSCalib:%.2fm%n",
                WP_REACH_RADIUS,Math.toDegrees(HEADING_TURN_THRESHOLD),GPS_CALIB_DIST);
            System.out.printf("  Heading: encoder-primary + GPS-calibrated (hybrid)%n");
            System.out.println("============================================");
            return;
        }
        if(!autonomousMode) return;

        if(docked){
            // Must keep calling executeTrack while docked so that:
            //  1. chargeOffset grows each tick  → battery percentage rises
            //  2. getBatteryPercentage() >= 99% triggers chargingDone=true
            //  3. exportData() and simulationComplete=true are set
            // Passing current GPS/cam is safe — dist < 0.30 keeps robot stopped.
            double gpsX=getGPSX(), gpsY=getGPSY(), cam=getTargetMaxScore();
            executeTrack(gpsX, gpsY, cam);
            return;
        }

        double s0=getSonarRange(0),s1=getSonarRange(1),s2=getSonarRange(2);
        double batt=getBatteryPercentage(), gpsX=getGPSX(), gpsY=getGPSY(), cam=getTargetMaxScore();
        updateDistance();

        if(batt<=0){setVel(0,0);if(!dataExported)exportData("BATTERY_DEAD");return;}

        if     (shouldAvoid(s0,s1,s2))     executeAvoid(s0,s1,s2);
        else if(shouldTrack(batt,cam))      executeTrack(gpsX,gpsY,cam);
        else if(shouldClean(batt))          executeClean(gpsX,gpsY);
        else if(shouldWander(batt))         executeWander();

        if(!activeBehaviour.equals(prevBehaviour)){
            logEvent("BEHAVIOUR_CHANGE",prevBehaviour+"->"+activeBehaviour+String.format(" gps=(%.2f,%.2f) batt=%.0f%%",gpsX,gpsY,batt));
            prevBehaviour=activeBehaviour; behavTransitions++;
        }
        if(!loggedBatt40&&batt<CLEAN_BATT_PCT){loggedBatt40=true;logEvent("BATTERY_WARN",String.format("below%.0f%% pos=(%.2f,%.2f)",CLEAN_BATT_PCT,gpsX,gpsY));}
        if(!loggedBatt20&&batt<TRACK_BATT_PCT){loggedBatt20=true;logEvent("BATTERY_CRIT",String.format("below%.0f%% pos=(%.2f,%.2f)",TRACK_BATT_PCT,gpsX,gpsY));}

        loopCount++;
        if(loopCount%LOG_INTERVAL==0){
            double el=getBatteryTime();
            dataLog.add(String.format("%.1f,%.2f,%.2f,%.1f,%.4f,%.4f,%.3f,%s,%.3f,%.3f,%.3f,%.3f,%s,%d",
                el,gpsX,gpsY,batt,getLeftWheelEnc(),getRightWheelEnc(),distanceTravelled,
                activeBehaviour,s0,s1,s2,cam,cleanPhase.name(),wpIndex));
            if(loopCount%(LOG_INTERVAL*10)==0)
                System.out.printf("[%s] GPS(%.2f,%.2f) WP:%d/%d Head:%.0f° Batt:%.0f%% Dist:%.1fm%n",
                    activeBehaviour,gpsX,gpsY,wpIndex,waypoints.size(),Math.toDegrees(headingEst),batt,distanceTravelled);
        }
    }
}
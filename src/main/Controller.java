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
import java.util.Arrays;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
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
 * Created by Theo Theodoridis.
 * Class    : Controller
 * Version  : v2.0
 * Date     : © Copyright 01/11/2020
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : Refactored with TLU-based Subsumption Architecture.
 **/

public class Controller {
    @FXML
    private Button btnConnect;
    @FXML
    private Button btnRight;
    @FXML
    private Button btnLeft;
    @FXML
    private Button btnBack;
    @FXML
    private Button btnForward;
    @FXML
    private Button btnStop;
    @FXML
    private Canvas canvasCamera;
    @FXML
    PixelWriter pw;
    @FXML
    private Label lblSensor0;
    @FXML
    private Label lblSensor1;
    @FXML
    private Label lblSensor2;
    @FXML
    private Label lblSensor3;
    @FXML
    private Label lblSensor4;
    @FXML
    private Label lblSensor5;
    @FXML
    private Label lbl;
    @FXML
    private Label lblGpsX;
    @FXML
    private Label lblGpsY;
    @FXML
    private Label lblGpsZ;
    @FXML
    private Label lblRightWheel;
    @FXML
    private Label lblLeftWheel;

    // Autonomous mode controls:
    @FXML
    private Button btnAutoMode;
    @FXML
    private Label lblBehavior;
    @FXML
    private Label lblBattery;
    @FXML
    private Label lblBatteryTime;
    @FXML
    private Label lblBatteryRate;
    @FXML
    private Label lblCharging;
    @FXML
    private Label lblDistance;
    @FXML
    private Button btnDrainFaster;
    @FXML
    private Button btnDrainSlower;
    @FXML
    private Button btnFastCharge;

    // TLU status labels:
    @FXML
    private Label lblTluAvoid;
    @FXML
    private Label lblTluTrack;
    @FXML
    private Label lblTluClean;
    @FXML
    private Label lblTluWander;
    @FXML
    private Label lblAvoidCount;
    @FXML
    private Label lblTrackCount;
    @FXML
    private Label lblCleanCount;
    @FXML
    private Label lblWanderCount;

    // Battery drain rate multiplier (1.0 = normal, 2.0 = double speed, etc.):
    private double batteryDrainRate = 1.0;
    /** Extra seconds consumed per tick when drainRate > 1 — applied manually in run(). */
    private int batteryBonusDrainCounter = 0;

    private String defaultButtonStyle;

    /**
     * Updates:
     **/

    private Color imageCamera[][];
    private double gpsValues[] = new double[3];
    private double sonarValues[] = new double[6];
    private double encoderValues[] = new double[2];

    private boolean runGPS = true;
    private boolean runCamera = true;
    private boolean runMotion = true;
    private boolean runSensors = true;
    private boolean runWheelEncoder = true;

    /**
     * Robot:
     **/

    private IntW cameraHandle = new IntW(1);
    private IntW leftWheelHandle = new IntW(1);
    private IntW rightWheelHandle = new IntW(1);

    private boolean running = false;
    private boolean firstCameraRead = true;
    private boolean firstSensor0Read = true;
    private boolean firstSensor1Read = true;
    private boolean firstSensor2Read = true;
    private boolean firstSensor3Read = true;
    private boolean firstSensor4Read = true;
    private boolean firstSensor5Read = true;
    private boolean firstLeftWheelCall = true;
    private boolean firstRightWheelCall = true;

    private char dir = 's'; // Direction.
    private int vel = 5;    // Velocity.

    /**
     * Camera:
     **/

    private double targetMinScore = 0.0;
    private double targetMaxScore = 0.0;

    private int resolutionCamera = 256;
    private CvPoint point = new CvPoint();
    private IntWA resolution = new IntWA(1);
    private CharWA image = new CharWA(resolutionCamera * resolutionCamera * 3);
    private char imageArray[] = new char[resolutionCamera * resolutionCamera * 3];
    private Color colorMatrix[][] = new Color[resolutionCamera][resolutionCamera];
    private BufferedImage bufferedImage = new BufferedImage(resolutionCamera, resolutionCamera, BufferedImage.TYPE_INT_RGB);

    /**
     * Wheel encoders:
     **/

    private float dxRight = 0;
    private float totalRightJointPosition = 0;
    private float currentRightJointPosition = 0;
    private float previousRightJointPosition = 0;
    private FloatW robotRightJointPosition = new FloatW(3);

    private float dxLeft = 0;
    private float totalLeftJointPosition = 0;
    private float currentLeftJointPosition = 0;
    private float previousLeftJointPosition = 0;
    private FloatW robotLeftJointPosition = new FloatW(3);

    /**
     * GPS:
     **/

    public static final double CHARGER_XCOORD = 1.78;  // The charger X coordinate.
    public static final double CHARGER_YCOORD = -0.78; // The charger Y coordinate.
    public static final double MAX_GPS_DIST = 5.0;   // The max Euclidean distance from the charger.

    /**
     * V-rep communication:
     **/

    private remoteApi vRep = new remoteApi();
    private int clientID = -1;

    /**
     * Timers:
     **/

    private int MAX_BATT_TIME = 60 * 45; // Default 45 mins battery time.
    private final int MAX_BATT_VOLT = 12;      // volts.

    private Timer motionTimer = new Timer();
    private Timer batteryTimer = new Timer();
    private Timer wanderTimer = new Timer();


    /********************************************************************************************************************
     *                                                   Battery::Methods                                               *
     ********************************************************************************************************************/

    public int getBatteryTime() {
        return (batteryTimer.getSec());
    }

    public void setBatteryTime(int min) {
        MAX_BATT_TIME = 60 * min;
        motionTimer.setSec(MAX_BATT_TIME);
        motionTimer.restart();
        wanderTimer.setSec(MAX_BATT_TIME);
        wanderTimer.restart();
    }

    public double getBatteryCapacity() {
        double v = (double) MAX_BATT_VOLT - Utils.map(batteryTimer.getSec(), 0, (double) MAX_BATT_TIME, 0, (double) MAX_BATT_VOLT);
        return ((v > 0.0) ? v : 0.0);
    }

    public double getBatteryPercentage() {
        // If currently charging, return the gradually increasing override value
        if (chargingOverridePct >= 0) {
            return Math.round(chargingOverridePct * 10.0) / 10.0;
        }
        double v = getBatteryCapacity();
        // Continuous percentage: linear map from 0V-12V to 0%-100%
        double pct = (v / (double) MAX_BATT_VOLT) * 100.0;
        return Math.max(0.0, Math.min(100.0, Math.round(pct * 10.0) / 10.0));
    }

    public boolean getBatteryState() {
        // If we are actively charging, battery is alive regardless of timer value
        if (chargingOverridePct >= 0) return true;
        double v = getBatteryCapacity();
        return ((v > 0.0) ? true : false);
    }


    /********************************************************************************************************************
     *                                                   Wheel::Methods                                                 *
     ********************************************************************************************************************/

    private double readRightWheelEnc() {
        if (firstRightWheelCall) {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_streaming);
            currentRightJointPosition = robotRightJointPosition.getValue();
            previousRightJointPosition = robotRightJointPosition.getValue();
            firstRightWheelCall = false;
            totalRightJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_buffer);
            currentRightJointPosition = robotRightJointPosition.getValue();
            dxRight = getAngleMinusAlpha(currentRightJointPosition, previousRightJointPosition);
            totalRightJointPosition += dxRight;
        }
        previousRightJointPosition = currentRightJointPosition;
        return (Math.round((totalRightJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private double readLeftWheelEnc() {
        if (firstLeftWheelCall) {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_streaming);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            previousLeftJointPosition = robotLeftJointPosition.getValue();
            firstLeftWheelCall = false;
            totalLeftJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_buffer);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            dxLeft = getAngleMinusAlpha(currentLeftJointPosition, previousLeftJointPosition);
            totalLeftJointPosition += dxLeft;
        }
        previousLeftJointPosition = currentLeftJointPosition;

        return (Math.round((totalLeftJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private float getAngleMinusAlpha(float angle, float alpha) {
        double sinAngle0 = Math.sin(angle);
        double sinAngle1 = Math.sin(alpha);
        double cosAngle0 = Math.cos(angle);
        double cosAngle1 = Math.cos(alpha);
        double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
        double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
        return ((float) Math.atan2(sin_da, cos_da));
    }

    public double getLeftWheelEnc() {
        return (encoderValues[0]);
    }

    public double getRightWheelEnc() {
        return (encoderValues[1]);
    }

    public int getEncoderNo() {
        return (2);
    }


    /********************************************************************************************************************
     *                                                   GPS::Methods                                                   *
     ********************************************************************************************************************/

    public double[] readGPS() {
        IntW baseHandle = new IntW(1);
        FloatWA position = new FloatWA(3);
        vRep.simxGetObjectHandle(clientID, "Roomba", baseHandle, remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectPosition(clientID, baseHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
        double positions[] = new double[position.getArray().length];

        positions[0] = Math.round((double) position.getArray()[0] * 100.0) / 100.0;
        positions[1] = Math.round((double) position.getArray()[1] * 100.0) / 100.0;
        positions[2] = Math.round((double) position.getArray()[2] * 100.0) / 100.0;

        return (positions);
    }

    public double getGPSX() {
        return (gpsValues[0]);
    }

    public double getGPSY() {
        return (gpsValues[1]);
    }

    public double getGPSZ() {
        return (gpsValues[2]);
    }

    public int getGPSNo() {
        return (3);
    }


    /********************************************************************************************************************
     *                                                   Ultrasonic::Methods                                            *
     ********************************************************************************************************************/

    private double readSonarRange(int sensor) {
        String sensorText = "";

        BoolW detectionState = new BoolW(false);
        FloatWA detectedPoint = new FloatWA(1); //Coordinates relatives to the sensor's frame
        IntW detectedObjectHandle = new IntW(1);
        FloatWA detectedSurfaceNormalVector = new FloatWA(1);

        IntW sensorHandle = new IntW(1);
        switch (sensor) {
            case 0:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor0", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor0Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor0Read = false;
                }
                break;
            case 1:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor1", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor1Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor1Read = false;
                }
                break;
            case 2:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor2", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor2Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor2Read = false;
                }
                break;
            case 3:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor3", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor3Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor3Read = false;
                }
                break;
            case 4:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor4", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor4Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor4Read = false;
                }
                break;
            case 5:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor5", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor5Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor5Read = false;
                }
                break;
        }

        float detectedPointXYZ[] = detectedPoint.getArray();
        double distance = Math.sqrt(Math.pow(detectedPointXYZ[0], 2) + Math.pow(detectedPointXYZ[1], 2) + Math.pow(detectedPointXYZ[2], 2));
        distance = Math.round(distance * 100d) / 100d;
        distance = Utils.getDecimal(distance, "0.0");
        distance = (distance >= 1.0) ? 1.0 : distance;
        distance = (distance == 0.0) ? 1.0 : distance;

        if (detectionState.getValue())
            return (distance);
        return (1.0);
    }

    private double[] readSonars() {
        for (int i = 0; i < getSonarNo(); i++)
            sonarValues[i] = readSonarRange(i);
        return (sonarValues);
    }

    private double[] getSonarRanges() {
        return (sonarValues);
    }

    public double getSonarRange(int sensor) {
        return (sonarValues[sensor]);
    }

    public int getSonarNo() {
        return (sonarValues.length);
    }


    /********************************************************************************************************************
     *                                                   Camera::Methods                                                *
     ********************************************************************************************************************/

    private Color[][] readCamera() {
        if (firstCameraRead) {
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_streaming);
            firstCameraRead = false;
        } else
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_buffer);
        return (imageToColor(image));
    }

    private Color[][] imageToColor(CharWA image) {
        imageArray = image.getArray();
        int index = 0;
        int r, g, b;
        Color color;

        for (int i = 0; i < resolutionCamera; i++)
            for (int j = 0; j < resolutionCamera; j++) {
                // Retrieve the RGB Values:
                r = (int) imageArray[index];
                g = (int) imageArray[index + 1];
                b = (int) imageArray[index + 2];
                color = new Color(r, g, b);
                colorMatrix[i][j] = color;

                bufferedImage.setRGB(i, j, new Color(r, g, b).getRGB());
                index += 3;
            }
        return (colorMatrix);
    }

    private int getGrayscale(BufferedImage img, int x, int y) {
        Color c = new Color(img.getRGB(x, y));
        int r = (int) (c.getRed() * 0.299);
        int g = (int) (c.getGreen() * 0.587);
        int b = (int) (c.getBlue() * 0.114);
        return ((r + g + b));
    }

    private static BufferedImage rotate(BufferedImage bimg, double angle, boolean color) {
        int imageType = -1;
        int w = bimg.getWidth();
        int h = bimg.getHeight();

        if (color) imageType = bimg.getType();
        else imageType = BufferedImage.TYPE_BYTE_GRAY;

        BufferedImage rotated = new BufferedImage(w, h, imageType);
        Graphics2D graphic = rotated.createGraphics();
        graphic.rotate(Math.toRadians(angle), w / 2, h / 2);
        graphic.drawImage(bimg, null, 0, 0);
        graphic.dispose();

        return (rotated);
    }

    public BufferedImage getImage() {
        return (rotate(bufferedImage, -90, false));
    }

    public int getImageWidth() {
        return (bufferedImage.getWidth());
    }

    public int getImageHeight() {
        return (bufferedImage.getHeight());
    }

    public int getImagePixel(int x, int y) {
        return (getGrayscale(bufferedImage, x, y));
    }

    public void setImagePixel(int x, int y, int rgb) {
        bufferedImage.setRGB(x, y, rgb + (rgb << 8) + (rgb << 16));
    }

    public int getTargetX() {
        return (point.x());
    }

    public int getTargetY() {
        return (point.y());
    }

    public double getTargetMinScore() {
        return (targetMinScore);
    }

    public double getTargetMaxScore() {
        return (targetMaxScore);
    }

    public void displayImage() {
        ImageViewer.display(getImage());
    }

    public void templateMatchingCV(BufferedImage image) {
        // [1]Convert BufferedImage to IplImage (grayscale source):
        IplImage src = Java2DFrameUtils.toIplImage(image);

        // [2]Load template marker image as grayscale:
        org.bytedeco.opencv.opencv_core.Mat tmpMat = imread("data/images/marker.jpg", IMREAD_GRAYSCALE);
        IplImage tmp = Java2DFrameUtils.toIplImage(tmpMat);

        // [3]The Correlation Image Result:
        IplImage result = cvCreateImage(cvSize(src.width() - tmp.width() + 1, src.height() - tmp.height() + 1), IPL_DEPTH_32F, 1);

        // [4]Template matching:
        cvMatchTemplate(src, tmp, result, CV_TM_CCOEFF_NORMED);

        // [5]Find min and max correlation values/locations:
        DoublePointer minVal = new DoublePointer(1);
        DoublePointer maxVal = new DoublePointer(1);
        CvPoint minLoc = new CvPoint();
        CvPoint maxLoc = new CvPoint();
        cvMinMaxLoc(result, minVal, maxVal, minLoc, maxLoc, null);

        targetMinScore = minVal.get();
        targetMaxScore = maxVal.get();

        // [6]Mark the template location:
        point.x(maxLoc.x() + tmp.width());
        point.y(maxLoc.y() + tmp.height());

        // [7]Draw the rectangle on the source image:
        cvRectangle(src, maxLoc, point, CvScalar.GRAY, 2, 8, 0);

        // [8]Display the image:
        ImageViewer.display(Java2DFrameUtils.toBufferedImage(src));
    }


    /********************************************************************************************************************
     *                                                   Motion::Methods                                                *
     ********************************************************************************************************************/

    public void forward() {
        resetButtonsStyle();
        btnForward.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'f';
    }

    public void backward() {
        resetButtonsStyle();
        btnBack.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'b';
    }

    public void left() {
        resetButtonsStyle();
        btnLeft.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'l';
    }

    public void right() {
        resetButtonsStyle();
        btnRight.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'r';
    }

    public void stop() {
        resetButtonsStyle();
        btnStop.setStyle("-fx-background-color: #7FFF00; ");
        dir = 's';
    }

    private void resetButtonsStyle() {
        btnRight.setStyle(defaultButtonStyle);
        btnStop.setStyle(defaultButtonStyle);
        btnLeft.setStyle(defaultButtonStyle);
        btnForward.setStyle(defaultButtonStyle);
        btnBack.setStyle(defaultButtonStyle);
    }

    public void setVel(float lVel, float rVel) {
        vRep.simxSetJointTargetVelocity(clientID, leftWheelHandle.getValue(), lVel, remoteApi.simx_opmode_oneshot);
        vRep.simxSetJointTargetVelocity(clientID, rightWheelHandle.getValue(), rVel, remoteApi.simx_opmode_oneshot);
    }

    public void move(float vel) {
        setVel(vel, vel);
    }

    public void move(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            move(vel);
        }
        stop();
    }

    public void turnSpot(float vel) {
        setVel(vel, -vel);
    }

    public void turnSpot(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSpot(vel);
        }
        stop();
    }

    public void turnSharp(float vel) {
        if (vel > 0) setVel(vel, 0);
        else setVel(0, -vel);
    }

    public void turnSharp(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSharp(vel);
        }
        stop();
    }

    public void turnSmooth(float vel) {
        if (vel > 0) setVel(vel, vel / 2);
        else setVel(-vel / 2, -vel);
    }

    public void turnSmooth(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSmooth(vel);
        }
        stop();
    }

    public void teleoperate(char dir, int vel) {
        switch (dir) {
            case 's':
                move(vel = 0);
                break;
            case 'f':
                move(+vel);
                break;
            case 'b':
                move(-vel);
                break;
            case 'r':
                turnSpot(+vel / 2);
                break;
            case 'l':
                turnSpot(-vel / 2);
                break;
        }
    }


    /********************************************************************************************************************
     *                                                   Generic::Methods                                               *
     ********************************************************************************************************************/

    public void connectToVrep() {
        clientID = vRep.simxStart("127.0.0.1", 20001, true, true, 5000, 5);
        if (clientID == -1) {
            btnConnect.setText("Failed");
            btnConnect.setStyle("-fx-background-color: #FF0000; ");
            vRep.simxFinish(clientID);
        } else {
            btnConnect.setStyle("-fx-background-color: #7FFF00; ");
            btnConnect.setText("Connected");
            setup();
        }
    }

    public void setup() {
        vRep.simxGetObjectHandle(clientID, "JointLeftWheel", leftWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "JointRightWheel", rightWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "Vision_sensor", cameraHandle, remoteApi.simx_opmode_blocking);

        defaultButtonStyle = btnForward.getStyle();
        pw = canvasCamera.getGraphicsContext2D().getPixelWriter();
        ImageViewer.open(resolutionCamera, resolutionCamera, "Camera");

        motionTimer.setSec(1);
        batteryTimer.setSec(MAX_BATT_TIME);
        wanderTimer.setSec(MAX_BATT_TIME);
        motionTimer.start();
        batteryTimer.start();
        wanderTimer.start();

        update.start();
        main.start();
    }


    /**
     * Method     : Controller::update()
     * Purpose    : To update the robot sensors and GUI.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : Runs in its own thread. Reads sensors, updates GUI labels, calls update().
     **/
    private Thread update = new Thread() {
        public void run() {
            setBatteryTime(45);
            while (true) {
                // [1]Update robot:
                if (runGPS) {
                    gpsValues = readGPS();
                }
                if (runSensors) {
                    sonarValues = readSonars();
                }
                if (runWheelEncoder) {
                    encoderValues[0] = readLeftWheelEnc();
                    encoderValues[1] = readRightWheelEnc();
                }
                if (runCamera) {
                    imageCamera = readCamera();
                }
                if (runMotion) {
                    teleoperate(dir, vel);
                }
                // Capture values for lambda:
                final double[] _gps = gpsValues.clone();
                final double[] _sonar = sonarValues.clone();
                final double[] _enc = encoderValues.clone();
                final String _behavior = currentBehavior;
                final double _batt = getBatteryPercentage();
                final int _battSec = getBatteryTime();
                final double _dist = distanceTravelled;
                final boolean _avoid = autonomousEnabled && _sonar.length >= 3 && shouldAvoid(_sonar[0], _sonar[1], _sonar[2]);
                final boolean _track = autonomousEnabled && shouldTrack(_batt, getTargetMaxScore());
                final boolean _clean = autonomousEnabled && shouldClean(_batt);
                final int _avoidCnt = avoidActivations;
                final int _trackCnt = trackActivations;
                final int _cleanCnt = cleanActivations;
                final int _wanderCnt = wanderActivations;
                final boolean _docked = dockedAtCharger;
                Platform.runLater(new Runnable() {
                    public void run() {
                        if (runGPS) {
                            lblGpsX.setText("X: " + _gps[0]);
                            lblGpsY.setText("Y: " + _gps[1]);
                            lblGpsZ.setText("Z: " + _gps[2]);
                        }
                        if (runSensors) {
                            lblSensor0.setText(Utils.getDecimal(_sonar[0], "0.00") + "m");
                            lblSensor1.setText(Utils.getDecimal(_sonar[1], "0.00") + "m");
                            lblSensor2.setText(Utils.getDecimal(_sonar[2], "0.00") + "m");
                            lblSensor3.setText(Utils.getDecimal(_sonar[3], "0.00") + "m");
                            lblSensor4.setText(Utils.getDecimal(_sonar[4], "0.00") + "m");
                            lblSensor5.setText(Utils.getDecimal(_sonar[5], "0.00") + "m");
                        }
                        if (runWheelEncoder) {
                            lblRightWheel.setText("Right: " + _enc[1]);
                            lblLeftWheel.setText("Left: " + _enc[0]);
                        }
                        // Behaviour & battery status:
                        if (lblBehavior != null) lblBehavior.setText(_behavior);
                        if (lblBattery != null) lblBattery.setText("Battery: " + (int)_batt + "%");
                        if (lblBatteryTime != null) lblBatteryTime.setText("Time: " + _battSec + "s");
                        if (lblDistance != null) lblDistance.setText("Distance: " + Utils.getDecimal(_dist, "0.00") + "m");
                        if (lblCharging != null) lblCharging.setText(_docked ? "CHARGING" : "");
                        if (lblBatteryRate != null) lblBatteryRate.setText("Rate: " + (int)batteryDrainRate + "x");
                        // TLU status:
                        if (lblTluAvoid != null) lblTluAvoid.setText("Avoid TLU: " + (_avoid ? "+1" : "-1"));
                        if (lblTluTrack != null) lblTluTrack.setText("Track TLU: " + (_track ? "+1" : "-1"));
                        if (lblTluClean != null) lblTluClean.setText("Clean TLU: " + (_clean ? "+1" : "-1"));
                        if (lblTluWander != null) lblTluWander.setText("Wander TLU: +1");
                        // Behaviour activation counts:
                        if (lblAvoidCount != null) lblAvoidCount.setText("Avoid: " + _avoidCnt);
                        if (lblTrackCount != null) lblTrackCount.setText("Track: " + _trackCnt);
                        if (lblCleanCount != null) lblCleanCount.setText("Clean: " + _cleanCnt);
                        if (lblWanderCount != null) lblWanderCount.setText("Wander: " + _wanderCnt);
                    }
                });

                // [2]Update custom code:
                update();

                // [3]Update battery:
                if (!getBatteryState()) {
                    System.err.println("Error: Robot out of battery...");
                    move(0, 1000);
                    running = false;
                    break;
                }
                Delay.ms(1);
            }
        }
    };

    /**
     * Method     : Controller::main()
     * Purpose    : To run the main control loop.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : Runs in its own thread. Calls main() which calls run() (subsumption coordinator).
     **/
    private Thread main = new Thread() {
        public void run() {
            while (true) {
                main();
                Delay.ms(1);
            }
        }
    };


    /********************************************************************************************************************
     *                                                                                                                  *
     *                                              STUDENT CODE                                                        *
     *                                                                                                                  *
     *  Implements a Subsumption Architecture with 4 behaviours:                                                        *
     *    1. AVOID  (Priority 1 - Highest) : Wall/obstacle avoidance using front ultrasonic sensors                     *
    *    2. TRACK  (Priority 2)           : Camera-guided charger tracking when battery <= 20% and target is detected  *
     *    3. CLEAN  (Priority 3)           : Systematic lawn-mower room coverage using GPS waypoints                    *
    *    4. WANDER (Priority 4 - Lowest)  : Random manoeuvres at scheduled intervals as fallback behaviour            *
     *                                                                                                                  *
     *  Each behaviour uses a TLU (Threshold Logic Unit) for activation decisions.                                      *
     *  Navigation uses GPS-based heading estimation with proportional steering control.                                *
     *  Only FRONT sensors (0, 1, 2) are used to avoid false detections from wide-angle side sensors.                   *
     *                                                                                                                  *
     ********************************************************************************************************************/

    // ==================== BEHAVIOURAL CONSTANTS ====================

    /** Room boundary coordinates (meters). Per assignment specification, room corners are at ±2.15m. */
    private static final double ROOM_MIN_X = -2.15;
    private static final double ROOM_MAX_X =  2.15;
    private static final double ROOM_MIN_Y = -2.15;
    private static final double ROOM_MAX_Y =  2.15;

    /** Distance between parallel cleaning lanes in the lawn-mower pattern (meters).
     *  0.35 m balances coverage density against number of 180° lane-end turns needed. */
    private static final double LANE_SPACING = 0.35;

    /** Distance threshold to consider a waypoint reached (meters).
     *  Increased from 0.25 to 0.40 so robot can "pass through" waypoints
     *  even after being deflected slightly by AVOID behavior. */
    private static final double WAYPOINT_TOLERANCE = 0.40;

    /** Minimum GPS movement required to update heading estimate (meters). Prevents noise from small GPS changes. */
    private static final double HEADING_MIN_MOVEMENT = 0.03;

    /** Distance from charger to consider docked (meters). */
    private static final double CHARGER_DOCK_RADIUS = 0.30;

    /** Coverage grid resolution (meters) for area-visited metric. */
    private static final double COVERAGE_RES = 0.20;

    /** Normal driving speed for cleaning and tracking (rad/s for wheel joints). */
    private static final float DRIVE_SPEED = 3.0f;

    /** Speed used during obstacle avoidance (rad/s). Slightly slower for safety but maintains forward progress. */
    private static final float AVOID_DRIVE_SPEED = 2.5f;

    /** Proportional gain for GPS-based steering correction.
     *  Reduced to 1.0: together with the steer clamp, this gives arc-based correction
     *  without any wheel going backward. At DRIVE_SPEED=3.0 and a 45° error,
     *  steer=0.79 with effectiveSpeed=2.12, keeping both wheels firmly forward. */
    private static final double STEERING_GAIN = 1.0;

    /** Proportional gain for reactive obstacle avoidance steering. */
    private static final double AVOID_STEER_GAIN = 2.0;

    /** Maximum sonar range at which obstacles trigger avoidance (meters).
     *  Obstacles beyond this distance are ignored. This prevents the TLU from
     *  firing when the robot is still far from actual obstructions (e.g. 0.5-0.7m
     *  from a wall). Only obstacles within AVOID_RANGE contribute to the TLU sum. */
    private static final double AVOID_RANGE = 0.45;

    /** Safety margin from room walls for cleaning waypoints (meters).
     *  Waypoints are inset by this amount so the cleaning path stays INSIDE
     *  the AVOID trigger zone. AVOID fires at ~0.45m from walls, so waypoints
     *  at ±1.65 are safely reachable without AVOID interference at lane ends. */
    private static final double WALL_MARGIN = 0.50;

    /** Cleaning area boundaries (inset by WALL_MARGIN). Used for coverage grid origin
     *  so that only accessible cells are counted toward coverage percentage. */
    private static final double CLEAN_MIN_X = ROOM_MIN_X + WALL_MARGIN;
    private static final double CLEAN_MIN_Y = ROOM_MIN_Y + WALL_MARGIN;
    private static final double CLEAN_MAX_X = ROOM_MAX_X - WALL_MARGIN;
    private static final double CLEAN_MAX_Y = ROOM_MAX_Y - WALL_MARGIN;

    /** Spacing of intermediate waypoints along each cleaning lane (meters).
     *  0.50 m keeps WPs achievable at driving speed without excessive waypoint count. */
    private static final double WP_ALONG_LANE_SPACING = 0.50;

    // ==================== AVOID TLU PARAMETERS ====================
    /** TLU weights for avoid behaviour. Inputs are range-gated proximity values for sensors 0, 1, 2.
     *  Centre sensor (1) is weighted higher because head-on collisions are more critical. */
    private static final double[] AVOID_WEIGHTS   = {0.5, 0.6, 0.5};
    /** TLU threshold for avoid behaviour. Tuned with range-gated proximity so avoidance
     *  triggers when obstacles are within ~0.35m (single sensor) or ~0.40m (multiple sensors). */
    private static final double   AVOID_THRESHOLD = 0.18;

    // ==================== TRACK TLU PARAMETERS ====================
    /** TLU weights for track behaviour.
     *  Sensors: [lowBattery, visualTargetDetected].
     *  TRACK fires only when both low battery and charger marker are visually detected. */
    private static final double[] TRACK_WEIGHTS   = {0.6, 0.8};
    /** TLU threshold for track behaviour. Requires low battery + visual detection. */
    private static final double   TRACK_THRESHOLD = 1.0;

    /** Assignment-compliant battery threshold for TRACK activation (<= 20%). */
    private static final double TRACK_BATT_TRIGGER_PCT = 20.0;

    /** Minimum template-matching score to consider charger marker visually detected. */
    private static final double TRACK_VISUAL_SCORE_THRESHOLD = 0.55;

    /** Steering gain for camera-based visual servoing during TRACK. */
    private static final double TRACK_CAMERA_STEER_GAIN = 1.8;

    /** Slow scan turn speed when TRACK is active but charger marker is temporarily lost. */
    private static final float TRACK_SEARCH_SPIN_SPEED = 1.0f;

    // ==================== CLEAN TLU PARAMETERS ====================
    /** TLU weight for clean behaviour. Input is 1.0 for battery in 40-100% range. */
    private static final double[] CLEAN_WEIGHTS   = {1.0};
    /** TLU threshold for clean behaviour. Fires when battery is sufficient for cleaning. */
    private static final double   CLEAN_THRESHOLD = 0.5;

    /** Assignment-compliant minimum battery threshold for CLEAN activation (>= 40%). */
    private static final double CLEAN_BATT_MIN_PCT = 40.0;


    // ==================== ENCODER DEAD-RECKONING CONSTANTS ====================

    /** Wheel base (distance between left and right wheels) in metres.
     *  Must be measured from the CoppeliaSim model joint positions. */
    private static final double WHEEL_BASE = 0.331;

    /** Wheel radius in metres. Must be measured from the CoppeliaSim model wheel object. */
    private static final double WHEEL_RADIUS = 0.072;

    /** GPS heading correction factor (0 = never correct, 1 = raw overwrite).
     *  With encoder dead-reckoning as the primary heading source, GPS serves only
     *  as a drift corrector. 0.30 means each GPS update nudges heading by 30%,
     *  correcting encoder drift before it accumulates into spinning. */
    private static final double HEADING_ALPHA = 0.30;

    /** Number of ticks to keep heading frozen after an AVOID manoeuvre ends.
     *  This prevents the corrupted AVOID heading from leaking into navigation. */
    private static final int AVOID_SETTLE_TICKS = 30;

    /** Spin-in-place speed for heading alignment in CLEAN behaviour (rad/s).
     *  2.5 rad/s completes a 180° lane-end turn in ~2.5 s — much faster than 1.5. */
    private static final float SPIN_SPEED = 2.5f;

    /** Heading error below which the robot is considered aligned (radians). 12°. */
    private static final double TURN_ALIGNED_THRESHOLD = Math.toRadians(12);

    /** Heading error that triggers an explicit spin-in-place (radians). 90°.
     *  Only errors > 90° require a dedicated spin (e.g. lane-end 180° reversals).
     *  Errors below 90° are handled smoothly by the P-controller arc steering,
     *  which avoids the spin-loop trap caused by a lower threshold and encoder drift. */
    private static final double TURN_TRIGGER_THRESHOLD = Math.toRadians(90);

    /** Maximum ticks for a spin-in-place before driving forward anyway. */
    private static final int MAX_TURN_TICKS = 200;


    // ==================== CLEAN STATE MACHINE ====================

    /** States for the CLEAN behaviour. */
    private enum CleanState { NAVIGATE_TO_WP, TURNING_TO_WP }
    private CleanState cleanState = CleanState.NAVIGATE_TO_WP;
    /** Tick counter used ONLY in TURNING_TO_WP state. */
    private int turnTickCount = 0;
    /** Navigate-only ticks on the current waypoint (turning ticks NOT counted).
     *  Ensures the stuck timer accurately reflects forward-progress time. */
    private int wpStuckCounter = 0;
    /** Maximum navigate ticks before skipping an unreachable WP (~21 s at 24 Hz). */
    private static final int MAX_WP_STUCK_TICKS = 500;


    // ==================== STATE VARIABLES ====================

    /** Whether autonomous mode has been enabled via the GUI button. False until user clicks "Auto Mode". */
    private boolean autonomousEnabled = false;

    /** Whether autonomous mode has been initialized. */
    private boolean autoInitialized = false;

    /** Charging state: true after docking; experiment ends after charge duration. */
    private boolean chargingComplete = false;
    private long chargeStartMs = 0;

    /** Battery percentage at the moment docking began. */
    private double chargeStartBattPct = 0.0;
    /** When >= 0, overrides getBatteryPercentage() during charging. Set to -1 when not charging. */
    private double chargingOverridePct = -1.0;
    private static final long CHARGE_DURATION_MS = 10000; // 10 seconds simulated charge

    /** Previous GPS coordinates for heading estimation. NaN means not yet initialized. */
    private double lastGpsX = Double.NaN;
    private double lastGpsY = Double.NaN;

    /** Estimated robot heading in radians (fused GPS + encoder). 0 = positive X direction. */
    private double robotHeading = 0.0;

    /** Whether we have a valid heading estimate (requires initial movement). */
    private boolean headingKnown = false;

    /** When true, heading estimation is frozen to prevent AVOID manoeuvres from
     *  corrupting the heading used by CLEAN/TRACK navigation. */
    private boolean headingFrozen = false;

    /** Countdown ticks after AVOID ends before heading is unfrozen.
     *  Allows the robot to resume straight-line motion before heading updates resume. */
    private int avoidSettleCounter = 0;

    /** Previous raw joint angles for encoder dead-reckoning (radians).
     *  Uses raw CoppeliaSim joint position, not cumulative revolutions,
     *  to handle backward rotation and ±π boundary correctly. */
    private double leftJointPrev = Double.NaN;
    private double rightJointPrev = Double.NaN;

    /** Whether the cleaning pattern has been started (first executeClean call performs nearest-WP search). */
    private boolean cleanPatternStarted = false;

    /** Cumulative count of waypoints reached (persists across pattern restarts). */
    private int totalWaypointsReached = 0;

    /** Pre-computed waypoints for the lawn-mower cleaning pattern. Each entry is {x, y}. */
    private double[][] cleaningWaypoints;

    /** Index of the current target waypoint in the cleaning pattern. */
    private int currentWaypointIdx = 0;

    /** Whether the robot has successfully docked at the charging station. */
    private boolean dockedAtCharger = false;

    /** Grid-based coverage map for visited cells. */
    private boolean[][] coverageGrid;
    private int coverageRows = 0;
    private int coverageCols = 0;
    private int coverageVisitedCount = 0;

    /** Name of the currently active behaviour (for logging and display). */
    private String currentBehavior = "INIT";

    /** Loop counter for throttled status printing. */
    private int loopCounter = 0;

    /** Print status to console every N ticks. At ~24 Hz tick rate, 1500 ≈ every 63 seconds. */
    private static final int STATUS_PRINT_INTERVAL = 1500;

    // ==================== WANDER TIMING (SCHEDULED INTERVALS) ====================

    /** Seconds between scheduled random manoeuvres during WANDER. */
    private static final int WANDER_INTERVAL_SEC = 4;

    /** Duration (seconds) of each scheduled random manoeuvre. */
    private static final int WANDER_TURN_DURATION_SEC = 2;

    /** Maximum steering magnitude applied during random manoeuvres. */
    private static final double WANDER_MAX_STEER = 1.2;

    /** Next second at which a random manoeuvre should start. */
    private int nextWanderTurnSec = -1;

    /** End second for the currently active random manoeuvre (-1 means inactive). */
    private int wanderTurnEndSec = -1;

    /** Current steering command used during a scheduled random manoeuvre. */
    private double wanderSteerCmd = 0.0;


    // ==================== METRICS & DATA COLLECTION ====================

    /** Total distance travelled estimated from GPS heading updates (meters). */
    private double distanceTravelled = 0.0;

    /** Count of how many ticks each behaviour was active. */
    private int avoidActivations = 0;
    private int trackActivations = 0;
    private int cleanActivations = 0;
    private int wanderActivations = 0;

    /** Collected experiment data rows for CSV export. */
    private List<String> dataLog = new ArrayList<>();

    /** Timestamp of when the autonomous run started (ms). */
    private long experimentStartMs = 0;

    /** Whether experiment data has been exported (to prevent duplicate writes). */
    private boolean experimentExported = false;


    /********************************************************************************************************************
     *                                          TLU (Threshold Logic Unit)                                              *
     *                                                                                                                  *
     *  The TLU takes a sensor vector s and a weight vector w. It computes the weighted sum                             *
     *  and compares it against threshold f:                                                                            *
     *    y = +1 (true)  if  Σ(w_i × s_i) > f                                                                         *
     *    y = -1 (false) otherwise                                                                                      *
     *  The weights and thresholds are set by intuition and tuned by trial and error.                                   *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::tlu()
     * Purpose    : Implements a Threshold Logic Unit for behaviour activation.
     * Parameters : - w_vec : Weight vector (one weight per sensor input).
     *              - s_vec : Sensor vector (normalised sensor readings).
     *              - f     : Activation threshold.
     * Returns    : True (+1) if TLU fires, False (-1) otherwise.
     * Notes      : Core decision mechanism for the subsumption architecture.
     **/
    public boolean tlu(double w_vec[], double s_vec[], double f) {
        double sum = 0.0;
        for (int i = 0; i < w_vec.length && i < s_vec.length; i++) {
            sum += w_vec[i] * s_vec[i];
        }
        return (sum > f);
    }


    /********************************************************************************************************************
     *                                         Heading Estimation (from GPS)                                            *
     *                                                                                                                  *
     *  Since the robot has no compass, heading is estimated from consecutive GPS readings.                             *
     *  The heading is only updated when the robot has moved more than HEADING_MIN_MOVEMENT                             *
     *  to avoid noise from GPS rounding (GPS rounds to 2 decimal places = ±0.005m).                                   *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::estimateHeadingFromGPS()
     * Purpose    : Updates the robot heading estimate from GPS position changes.
     *              Uses an exponential low-pass filter (HEADING_ALPHA) to smooth
     *              noisy GPS deltas, and respects the headingFrozen flag to prevent
     *              AVOID manoeuvres from corrupting the navigation heading.
     * Parameters : gpsX, gpsY - current GPS coordinates.
     * Returns    : Nothing. Updates robotHeading and headingKnown fields.
     **/
    private void estimateHeadingFromGPS(double gpsX, double gpsY) {
        // Guard against garbage GPS values from uninitialized V-REP streaming buffer
        if (Math.abs(gpsX) > 100.0 || Math.abs(gpsY) > 100.0) return;

        if (Double.isNaN(lastGpsX)) {
            lastGpsX = gpsX;
            lastGpsY = gpsY;
            return;
        }

        double dx = gpsX - lastGpsX;
        double dy = gpsY - lastGpsY;
        double dist = Math.hypot(dx, dy);

        // Reject implausibly large jumps (GPS glitch or V-REP init artifact)
        if (dist > 5.0) {
            lastGpsX = gpsX;
            lastGpsY = gpsY;
            return;
        }

        if (dist < HEADING_MIN_MOVEMENT) return;

        // Always update distance and baseline position
        lastGpsX = gpsX;
        lastGpsY = gpsY;
        distanceTravelled += dist;

        // GPS serves as a CORRECTION to encoder-based heading, not primary source.
        // Only apply when heading is not frozen (i.e. not during or just after AVOID).
        if (!headingFrozen) {
            double gpsHeading = Math.atan2(dy, dx);

            if (!headingKnown) {
                // First valid heading — accept GPS value immediately
                robotHeading = gpsHeading;
                headingKnown = true;
            } else {
                // Small correction factor: encoder leads, GPS gently nudges toward truth
                // HEADING_ALPHA = 0.08 → only 8% weight to GPS per update
                double err = normalizeAngle(gpsHeading - robotHeading);
                robotHeading = normalizeAngle(robotHeading + HEADING_ALPHA * err);
            }
        }
    }

    /**
     * Method     : Controller::updateHeadingFromEncoders()
     * Purpose    : Supplements GPS heading estimation with wheel-encoder dead-reckoning.
     *              Encoders provide smooth, lag-free heading updates during slow turns
     *              and pivots where GPS deltas are too small to be useful.
     * Parameters : leftEnc, rightEnc - current wheel encoder readings (revolutions).
     * Returns    : Nothing. Updates robotHeading from encoder deltas.
     * Notes      : The encoder change dθ = (dRight − dLeft) / WHEEL_BASE gives the
     *              change in heading. This is applied every tick regardless of
     *              headingFrozen, because encoders directly measure wheel rotation
     *              and don't suffer the same corruption problem as GPS during AVOID.
     *              However, during headingFrozen we only apply encoder heading if
     *              the robot is purposefully spinning (TURNING_TO_WP state).
     **/
    private void updateHeadingFromEncoders(double leftJoint, double rightJoint) {
        if (Double.isNaN(leftJointPrev)) {
            leftJointPrev = leftJoint;
            rightJointPrev = rightJoint;
            return;
        }

        // CoppeliaSim returns absolute joint angle in radians.
        // Compute signed angular delta with ±π wrapping to handle backward rotation
        // and the ±π boundary correctly.
        double dLeft  = normalizeAngle(leftJoint  - leftJointPrev);
        double dRight = normalizeAngle(rightJoint - rightJointPrev);

        leftJointPrev  = leftJoint;
        rightJointPrev = rightJoint;

        // Convert joint-angle delta (radians) to linear distance (metres)
        double distLeft  = dLeft  * WHEEL_RADIUS;
        double distRight = dRight * WHEEL_RADIUS;

        // Differential drive heading update: Δθ = (distRight − distLeft) / wheelBase
        double dTheta = (distRight - distLeft) / WHEEL_BASE;

        // Encoders are the PRIMARY heading source — always integrate.
        // (GPS only applies a small correction via HEADING_ALPHA = 0.08)
        robotHeading = normalizeAngle(robotHeading + dTheta);

        if ((Math.abs(dLeft) + Math.abs(dRight)) > 0.0001) {
            headingKnown = true;
        }
    }

    /**
     * Method     : Controller::normalizeAngle()
     * Purpose    : Normalises an angle to the range [-π, +π].
     * Parameters : angle - the angle in radians.
     * Returns    : The normalised angle.
     **/
    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /** Marks the coverage grid cell for the given GPS position.
     *  Grid origin is at (CLEAN_MIN_X, CLEAN_MIN_Y) so only accessible cleaning
     *  area cells are tracked — wall-margin cells are excluded from coverage %. */
    private void markCoverage(double gpsX, double gpsY) {
        if (coverageGrid == null) return;
        int col = (int)Math.floor((gpsX - CLEAN_MIN_X) / COVERAGE_RES);
        int row = (int)Math.floor((gpsY - CLEAN_MIN_Y) / COVERAGE_RES);
        if (row < 0 || col < 0 || row >= coverageRows || col >= coverageCols) return;
        if (!coverageGrid[row][col]) {
            coverageGrid[row][col] = true;
            coverageVisitedCount++;
        }
    }

    /** Returns coverage percent (0-100) of visited cells. */
    private double getCoveragePercent() {
        if (coverageGrid == null || coverageRows == 0 || coverageCols == 0) return 0.0;
        int total = coverageRows * coverageCols;
        if (total == 0) return 0.0;
        return (100.0 * coverageVisitedCount) / total;
    }


    /********************************************************************************************************************
     *                                     Waypoint Generation (Lawn-Mower Pattern)                                     *
     *                                                                                                                  *
     *  Generates a boustrophedon (zigzag) path covering the room:                                                     *
     *    Lane 0: left boundary → right boundary (at y = ROOM_MIN_Y)                                                   *
     *    Lane 1: right boundary → left boundary (at y = ROOM_MIN_Y + LANE_SPACING)                                    *
     *    Lane 2: left → right (at y = ROOM_MIN_Y + 2 × LANE_SPACING)                                                 *
     *    ... and so on until the top boundary.                                                                         *
     *  This ensures systematic coverage of the entire room area.                                                       *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::initCleaningWaypoints()
     * Purpose    : Pre-computes a flat boustrophedon (lawn-mower) waypoint list covering
     *              the entire cleaning area. Lanes run parallel to the X axis, alternating
     *              direction row by row from bottom (y=CLEAN_MIN_Y) to top (y=CLEAN_MAX_Y).
     *              The robot navigates this flat list sequentially: WP 0 → 1 → … → N-1,
     *              then restarts. On first call, it starts from the waypoint nearest to
     *              its current GPS position.
     * Returns    : Nothing. Sets cleaningWaypoints array and coverage grid.
     **/
    private void initCleaningWaypoints() {
        // Q1 ONLY: bottom-right quadrant (x: 0..+1.65, y: -1.65..0).
        // This is the quadrant nearest to the charger spawn point, so the robot
        // starts here and systematically covers it before battery drops.
        // Keeping the area small avoids long initial sprints and large heading errors.
        double xMin = 0.0;          // Q1 left boundary (room centre)
        double xMax = CLEAN_MAX_X;  // +1.65
        double yMin = CLEAN_MIN_Y;  // -1.65
        double yMax = 0.0;          // Q1 top boundary (room centre)

        List<double[]> wps = new ArrayList<>();
        boolean leftToRight = true;  // Lane 0 goes left→right (x: 0 → +1.65)

        for (double laneY = yMin; laneY <= yMax + 0.001; laneY += LANE_SPACING) {
            double clampedY = Math.min(laneY, yMax);
            double startX   = leftToRight ? xMin : xMax;
            double dir      = leftToRight ? 1.0 : -1.0;
            double laneDist = xMax - xMin;
            int steps = (int) Math.ceil(laneDist / WP_ALONG_LANE_SPACING);

            for (int i = 0; i <= steps; i++) {
                double x = startX + dir * Math.min(i * WP_ALONG_LANE_SPACING, laneDist);
                wps.add(new double[]{x, clampedY});
            }
            leftToRight = !leftToRight;
        }

        cleaningWaypoints = wps.toArray(new double[0][]);
        int numLanes = (int) Math.ceil((yMax - yMin) / LANE_SPACING) + 1;
        System.out.printf("[CLEAN] Q1 ONLY: %d waypoints across %d lanes  lane=%.2fm  wp=%.2fm%n",
                cleaningWaypoints.length, numLanes, LANE_SPACING, WP_ALONG_LANE_SPACING);
        System.out.printf("[CLEAN] Q1 bounds: x=[%.2f..%.2f]  y=[%.2f..%.2f]%n",
                xMin, xMax, yMin, yMax);

        // Build coverage grid (full room for overall coverage reporting)
        coverageCols = (int) Math.ceil((CLEAN_MAX_X - CLEAN_MIN_X) / COVERAGE_RES);
        coverageRows = (int) Math.ceil((CLEAN_MAX_Y - CLEAN_MIN_Y) / COVERAGE_RES);
        coverageGrid = new boolean[coverageRows][coverageCols];
        coverageVisitedCount = 0;
    }


    /********************************************************************************************************************
     *                                      GPS Navigation (Proportional Steering)                                      *
     *                                                                                                                  *
     *  Steers the robot toward a target point using proportional control:                                              *
     *    1. Compute desired heading = atan2(targetY - gpsY, targetX - gpsX)                                           *
     *    2. Compute heading error = desired - current (normalised to [-π, π])                                          *
     *    3. Apply steering: steer = -Kp × error                                                                       *
     *       (negative because positive error = target left, but leftVel > rightVel = turn right)                       *
     *    4. Set wheel velocities: left = speed + steer, right = speed - steer                                         *
     *                                                                                                                  *
     *  If heading is not yet known (robot hasn't moved enough), drives straight to establish heading.                  *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::steerTowardPoint()
     * Purpose    : Navigates the robot toward a target GPS coordinate using proportional steering.
     * Parameters : targetX, targetY - destination coordinates.
     *              gpsX, gpsY       - current robot position.
     *              speed            - base driving speed (rad/s).
     * Returns    : True if the robot has arrived at the target (within WAYPOINT_TOLERANCE), false otherwise.
     **/
    private boolean steerTowardPoint(double targetX, double targetY, double gpsX, double gpsY, float speed) {
        double dx = targetX - gpsX;
        double dy = targetY - gpsY;
        double dist = Math.sqrt(dx * dx + dy * dy);

        // Check if arrived at target
        if (dist < WAYPOINT_TOLERANCE) {
            return true;
        }

        // If heading not yet established, drive straight forward to build up GPS delta
        if (!headingKnown) {
            setVel(speed, speed);
            return false;
        }

        // Compute desired heading toward target
        double desiredHeading = Math.atan2(dy, dx);
        double error = normalizeAngle(desiredHeading - robotHeading);

        // Cosine speed scaling: reduce forward speed when heading error is large.
        // At 0° error → full speed. At 90° error → minimum crawl. Smooth in between.
        // This prevents wide diagonal arcs when heading is misaligned.
        double clampedError = Math.min(Math.abs(error), Math.PI / 2.0);
        float effectiveSpeed = (float)(speed * Math.cos(clampedError));
        effectiveSpeed = Math.max(0.3f, effectiveSpeed);  // Never fully stop

        // Proportional steering: negative sign because:
        //   error > 0 (target to the left) → need to turn left → rightVel > leftVel → steer < 0
        //   error < 0 (target to the right) → need to turn right → leftVel > rightVel → steer > 0
        double steer = -STEERING_GAIN * error;

        // Clamp steering to prevent wheel reversal during normal navigation
        steer = Math.max(-effectiveSpeed, Math.min(effectiveSpeed, steer));

        float leftVel  = (float)(effectiveSpeed + steer);
        float rightVel = (float)(effectiveSpeed - steer);
        setVel(leftVel, rightVel);
        return false;
    }


    /********************************************************************************************************************
     *                                           BEHAVIOUR 1: AVOID                                                     *
     *  Priority    : 1 (Highest — subsumes all other behaviours)                                                       *
     *  Sensors     : Ultrasonic sensors 0 (front-left), 1 (front-centre), 2 (front-right)                             *
     *  Activation  : TLU fires when front obstacle proximity exceeds threshold                                         *
     *  Action      : Proportional reactive steering away from the obstacle                                             *
     *  Note        : Side sensors (3, 4, 5) are NOT used because their wide detection angle                            *
     *                causes false positives that prevent forward movement.                                              *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::shouldAvoid()
     * Purpose    : TLU activation check for the AVOID behaviour.
     * Parameters : s0, s1, s2 - front sonar readings (0-1m, 1.0 = clear).
     * Returns    : True if obstacle avoidance should activate.
     **/
    private boolean shouldAvoid(double s0, double s1, double s2) {
        // Range-gated proximity: obstacles beyond AVOID_RANGE contribute 0.
        // This prevents avoidance from firing when the robot is 0.5-0.7m from a wall
        // (which happens constantly when driving parallel to walls).
        double p0 = Math.max(0.0, (AVOID_RANGE - s0) / AVOID_RANGE);
        double p1 = Math.max(0.0, (AVOID_RANGE - s1) / AVOID_RANGE);
        double p2 = Math.max(0.0, (AVOID_RANGE - s2) / AVOID_RANGE);
        double[] proximitySensors = {p0, p1, p2};
        return tlu(AVOID_WEIGHTS, proximitySensors, AVOID_THRESHOLD);
    }

    /**
     * Method     : Controller::executeAvoid()
     * Purpose    : Executes the AVOID behaviour by steering away from obstacles.
     * Parameters : s0, s1, s2 - front sonar readings.
     * Notes      : ALWAYS TURNS RIGHT when obstacle detected (consistent right-turn bias).
     *              This creates predictable wall-following behavior.
     **/
    private void executeAvoid(double s0, double s1, double s2) {
        avoidActivations++;
        currentBehavior = "AVOID";

        // Freeze heading estimation so AVOID steering doesn't corrupt the
        // navigation heading used by CLEAN/TRACK after the obstacle is cleared.
        headingFrozen = true;
        avoidSettleCounter = AVOID_SETTLE_TICKS;  // Keep frozen for N ticks after AVOID ends

        // Range-gated proximity (consistent with shouldAvoid)
        double leftProx   = Math.max(0.0, (AVOID_RANGE - s0) / AVOID_RANGE);
        double centerProx = Math.max(0.0, (AVOID_RANGE - s1) / AVOID_RANGE);
        double rightProx  = Math.max(0.0, (AVOID_RANGE - s2) / AVOID_RANGE);

        double steer;
        float speed;

        if (s1 < 0.15) {
            // EMERGENCY: Obstacle very close directly ahead (< 15cm).
            // Always turn RIGHT (positive steer) at low speed.
            speed = 0.5f;
            steer = 3.5;    // Hard right turn
        } else if (centerProx > 0.5) {
            // MODERATE: Center obstacle within ~23cm. Moderate right turn with reduced speed.
            speed = 1.5f;
            steer = 2.5;    // Medium right turn
        } else {
            // GENTLE: Proportional right-biased steering while maintaining forward speed.
            // Use average proximity to determine turn intensity (higher proximity = sharper right turn).
            speed = AVOID_DRIVE_SPEED;
            double avgProx = (leftProx + centerProx + rightProx) / 3.0;
            steer = avgProx * AVOID_STEER_GAIN;  // Positive = right turn
        }

        setVel((float)(speed + steer), (float)(speed - steer));
    }


    /********************************************************************************************************************
     *                                           BEHAVIOUR 2: TRACK                                                     *
     *  Priority    : 2 (Second highest — subsumes CLEAN and WANDER)                                                    *
     *  Sensors     : Battery (percentage), GPS (distance to dock), Camera (marker detection + centering)              *
     *  Activation  : TLU fires when battery <= 20% AND charger marker is visually detected                             *
     *  Action      : Camera-based visual servoing toward the charger marker                                             *
     *  Note        : GPS is used only to detect final docking range (< CHARGER_DOCK_RADIUS).                           *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::shouldTrack()
     * Purpose    : TLU activation check for the TRACK behaviour.
     * Parameters : batteryPct - current battery percentage (continuous 0.0–100.0).
     *              targetScore - camera template matching score for charger marker.
     * Returns    : True only when battery is critical and visual target is detected.
     **/
    private boolean shouldTrack(double batteryPct, double targetScore) {
        double lowBattery = (batteryPct <= TRACK_BATT_TRIGGER_PCT) ? 1.0 : 0.0;
        double visualDetected = (targetScore >= TRACK_VISUAL_SCORE_THRESHOLD) ? 1.0 : 0.0;
        double[] sensors = { lowBattery, visualDetected };
        return tlu(TRACK_WEIGHTS, sensors, TRACK_THRESHOLD);
    }

    /**
     * Method     : Controller::executeTrack()
     * Purpose    : Executes the TRACK behaviour via camera-based marker tracking.
     * Parameters : gpsX, gpsY - current GPS coordinates.
     *              targetScore - camera template matching score for charger marker.
     * Notes      : Uses getTargetX(), getTargetY(), getTargetMaxScore() for visual control.
     *              GPS is used only for final docking distance check.
     **/
    private void executeTrack(double gpsX, double gpsY, double targetScore) {
        trackActivations++;
        currentBehavior = "TRACK";

        double dist = Utils.getEuclidean(gpsX, gpsY, CHARGER_XCOORD, CHARGER_YCOORD);

        // Check if docked
        if (dist < CHARGER_DOCK_RADIUS) {
            setVel(0, 0);
            dockedAtCharger = true;

            // Begin simulated charging once docked
            if (!chargingComplete) {
                if (chargeStartMs == 0) {
                    chargeStartMs = System.currentTimeMillis();
                    chargeStartBattPct = getBatteryPercentage();
                    batteryTimer.suspend(); // Freeze battery drain while charging
                    chargingOverridePct = chargeStartBattPct; // Start override at current level
                    System.out.println("============================================");
                    System.out.println("  DOCKED AT CHARGING STATION");
                    System.out.println("  Final position: (" + gpsX + ", " + gpsY + ")");
                    System.out.println("  Distance to charger: " + Utils.getDecimal(dist, "0.00") + "m");
                    System.out.println("  Battery (before charge): " + String.format("%.1f", chargeStartBattPct) + "%");
                    System.out.println("  Charging for " + (CHARGE_DURATION_MS/1000) + "s...");
                    System.out.println("============================================");
                }

                long elapsedCharge = System.currentTimeMillis() - chargeStartMs;
                double chargeProgress = Math.min(1.0, (double) elapsedCharge / CHARGE_DURATION_MS);

                // Gradually increase battery from dock-time % to 100% over charge duration
                chargingOverridePct = chargeStartBattPct + (100.0 - chargeStartBattPct) * chargeProgress;

                if (elapsedCharge >= CHARGE_DURATION_MS) {
                    // Fully charged: reset battery timer and clear override
                    batteryTimer.restart(); // counter=0 = full battery
                    chargingOverridePct = -1.0; // Let normal getBatteryPercentage() take over
                    chargeStartMs = 0;
                    chargingComplete = true;
                    System.out.println("[CHARGE] Battery restored to 100%. Ending experiment.");
                    if (!experimentExported) {
                        exportExperimentData("DOCKED_AND_CHARGED");
                    }
                }
            }
            return;
        }

        // Visual target lost while in TRACK: spin slowly to reacquire marker.
        if (targetScore < TRACK_VISUAL_SCORE_THRESHOLD) {
            setVel(TRACK_SEARCH_SPIN_SPEED, -TRACK_SEARCH_SPIN_SPEED);
            return;
        }

        // Camera-based approach: center target horizontally in image.
        int imgW = getImageWidth();
        int imgH = getImageHeight();
        double imgCenterX = imgW / 2.0;
        int targetX = getTargetX();
        int targetY = getTargetY();
        double normalizedXError = (targetX - imgCenterX) / imgCenterX;
        normalizedXError = Math.max(-1.0, Math.min(1.0, normalizedXError));

        // Keep using GPS distance to slow down near dock.
        float speed;
        if (dist < 0.6) {
            speed = DRIVE_SPEED * 0.4f;  // Very slow final approach
        } else if (dist < 1.2) {
            speed = DRIVE_SPEED * 0.7f;  // Medium approach
        } else {
            speed = DRIVE_SPEED;          // Full speed when far
        }

        // Use vertical position as an extra speed damping factor.
        double normalizedY = Math.max(0.0, Math.min(1.0, targetY / (double) imgH));
        speed = (float) (speed * Math.max(0.6, normalizedY));

        double steer = TRACK_CAMERA_STEER_GAIN * normalizedXError;
        steer = Math.max(-speed, Math.min(speed, steer));

        setVel((float) (speed + steer), (float) (speed - steer));
    }


    /********************************************************************************************************************
     *                                           BEHAVIOUR 3: CLEAN                                                     *
     *  Priority    : 3 (Subsumes only WANDER)                                                                          *
     *  Sensors     : Battery (percentage), GPS (position) for waypoint navigation                                      *
    *  Activation  : TLU fires when battery percentage is in 40–100%                                                   *
     *  Action      : Follow a pre-computed lawn-mower waypoint path across the room                                    *
     *  Note        : Pattern restarts from the beginning if all waypoints are visited.                                  *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::shouldClean()
     * Purpose    : TLU activation check for the CLEAN behaviour.
     * Parameters : batteryPct - current battery percentage.
     * Returns    : True if battery is within assignment CLEAN band (40–100%).
     **/
    private boolean shouldClean(double batteryPct) {
        double[] sensors = { (batteryPct >= CLEAN_BATT_MIN_PCT) ? 1.0 : 0.0 };
        return tlu(CLEAN_WEIGHTS, sensors, CLEAN_THRESHOLD);
    }

    /**
     * Method     : Controller::executeClean()
     * Purpose    : Executes the CLEAN behaviour using QUADRANT-BASED navigation.
     *              Room is divided into 4 quadrants. The robot completes all waypoints
     *              in one quadrant before moving to the next.
     *
     *              KEY DESIGN:
     *              - On each quadrant entry, finds the NEAREST waypoint to the robot's
     *                current position (not WP 0 which may be in a far corner).
     *              - Navigates sequentially from that nearest WP, wrapping around within
     *                the quadrant's WP array until all are handled.
     *              - Tracks handled WPs (reached + skipped) separately from the index.
     *              - If robot drifts outside quadrant bounds (due to AVOID), steers it
     *                back toward the target WP which is inside the quadrant.
     *
     * Parameters : gpsX, gpsY - current GPS coordinates.
     **/
    private void executeClean(double gpsX, double gpsY) {
        cleanActivations++;
        currentBehavior = "CLEAN";

        // ── INITIALISE on first call: start from nearest WP to current position ──
        if (!cleanPatternStarted) {
            cleanPatternStarted = true;
            cleanState = CleanState.NAVIGATE_TO_WP;
            turnTickCount = 0;
            wpStuckCounter = 0;
            currentWaypointIdx = findNearestWaypoint(gpsX, gpsY);
            System.out.printf("[CLEAN] Starting at WP %d/%d (%.2f, %.2f)%n",
                    currentWaypointIdx + 1, cleaningWaypoints.length,
                    cleaningWaypoints[currentWaypointIdx][0],
                    cleaningWaypoints[currentWaypointIdx][1]);
        }

        double[] wp = cleaningWaypoints[currentWaypointIdx];
        double dx          = wp[0] - gpsX;
        double dy          = wp[1] - gpsY;
        double distance    = Math.sqrt(dx * dx + dy * dy);
        double desiredHdg  = Math.atan2(dy, dx);

        // ── TURNING STATE: spin in place until aligned ────────────────────
        if (cleanState == CleanState.TURNING_TO_WP) {
            turnTickCount++;
            if (!headingKnown) {
                // Drive slowly forward until heading becomes known
                setVel(DRIVE_SPEED * 0.4f, DRIVE_SPEED * 0.4f);
                return;
            }
            double headingError = normalizeAngle(desiredHdg - robotHeading);
            if (Math.abs(headingError) < TURN_ALIGNED_THRESHOLD || turnTickCount > MAX_TURN_TICKS) {
                // Aligned (or timed out) — switch back to navigate
                cleanState = CleanState.NAVIGATE_TO_WP;
                headingFrozen = false;
                turnTickCount = 0;
            } else {
                // Spin toward target: positive error = target left = spin left
                if (headingError > 0) {
                    setVel(-SPIN_SPEED, SPIN_SPEED);
                } else {
                    setVel(SPIN_SPEED, -SPIN_SPEED);
                }
                return;  // Do NOT increment wpStuckCounter during turns
            }
        }

        // ── WAYPOINT REACHED ─────────────────────────────────────────────
        if (distance < WAYPOINT_TOLERANCE) {
            wpStuckCounter = 0;
            totalWaypointsReached++;
            System.out.printf("[CLEAN] WP %d/%d reached (%.2f,%.2f)  total=%d%n",
                    currentWaypointIdx + 1, cleaningWaypoints.length,
                    wp[0], wp[1], totalWaypointsReached);

            // Advance to next WP (wrap around and restart)
            currentWaypointIdx++;
            if (currentWaypointIdx >= cleaningWaypoints.length) {
                currentWaypointIdx = 0;
                System.out.println("[CLEAN] All waypoints complete — restarting pattern.");
            }

            // If next WP requires a turn > TURN_TRIGGER_THRESHOLD, spin first
            double[] nextWp   = cleaningWaypoints[currentWaypointIdx];
            double nextDx     = nextWp[0] - gpsX;
            double nextDy     = nextWp[1] - gpsY;
            double nextHdg    = Math.atan2(nextDy, nextDx);
            double nextError  = headingKnown ? Math.abs(normalizeAngle(nextHdg - robotHeading)) : 0.0;
            if (nextError > TURN_TRIGGER_THRESHOLD) {
                cleanState = CleanState.TURNING_TO_WP;
                turnTickCount = 0;
            }
            return;
        }

        // ── STUCK DETECTION (navigate ticks only, not turn ticks) ────────
        wpStuckCounter++;
        if (wpStuckCounter > MAX_WP_STUCK_TICKS) {
            System.out.printf("[CLEAN] Stuck on WP %d for %d navigate-ticks — skipping%n",
                    currentWaypointIdx + 1, wpStuckCounter);
            wpStuckCounter = 0;
            currentWaypointIdx = (currentWaypointIdx + 1) % cleaningWaypoints.length;
            cleanState = CleanState.NAVIGATE_TO_WP;
            return;
        }

        // ── DRIVE TOWARD WAYPOINT ────────────────────────────────────────
        if (!headingKnown) {
            setVel(DRIVE_SPEED * 0.4f, DRIVE_SPEED * 0.4f);
            return;
        }

        double headingError = normalizeAngle(desiredHdg - robotHeading);

        // No mid-navigate spin re-trigger — P-controller arcs for errors up to 90°.
        // Removing this check eliminates the spin-loop trap where small encoder drift
        // repeatedly triggered spinning before the robot could make forward progress.

        // Proportional steering with cosine speed scaling
        double clampedError   = Math.min(Math.abs(headingError), Math.PI / 2.0);
        float  effectiveSpeed = (float)(DRIVE_SPEED * Math.cos(clampedError));
        effectiveSpeed        = Math.max(0.4f, effectiveSpeed);
        double steer          = -STEERING_GAIN * headingError;
        steer                 = Math.max(-effectiveSpeed, Math.min(effectiveSpeed, steer));
        setVel((float)(effectiveSpeed + steer), (float)(effectiveSpeed - steer));
    }

    /**
     * Returns the index of the waypoint in cleaningWaypoints nearest to (gpsX, gpsY).
     * Called once at the start of each CLEAN session to avoid long initial sprints.
     */
    private int findNearestWaypoint(double gpsX, double gpsY) {
        int    nearestIdx  = 0;
        double nearestDist = Double.MAX_VALUE;
        for (int i = 0; i < cleaningWaypoints.length; i++) {
            double d = Math.hypot(gpsX - cleaningWaypoints[i][0], gpsY - cleaningWaypoints[i][1]);
            if (d < nearestDist) { nearestDist = d; nearestIdx = i; }
        }
        return nearestIdx;
    }


    /********************************************************************************************************************
     *                                           BEHAVIOUR 4: WANDER                                                    *
     *  Priority    : 4 (Lowest — fallback behaviour)                                                                   *
     *  Sensors     : Battery (implicit — only active when other behaviours don't fire)                                  *
     *  Activation  : Always active (fallback)                                                                           *
    *  Action      : Straight drive with random manoeuvres at scheduled intervals                                       *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::executeWander()
     * Purpose    : Executes WANDER with timer-scheduled random manoeuvres.
     * Notes      : Uses Timer.getSec() to start short random turns at fixed intervals.
     **/
    private void executeWander() {
        wanderActivations++;
        currentBehavior = "WANDER";

        // ── CLEAN → WANDER TRANSITION: abandon all waypoint state ────────────
        // When battery drops below CLEAN_BATT_MIN_PCT (40%), CLEAN stops being called.
        // cleanPatternStarted may still be true from a prior CLEAN run, which would
        // cause CLEAN to resume from a stale mid-pattern position after recharging.
        // Reset everything here so the next CLEAN entry starts fresh from the
        // robot's post-wander position (nearest WP search will run again).
        if (cleanPatternStarted) {
            cleanPatternStarted = false;
            cleanState = CleanState.NAVIGATE_TO_WP;
            wpStuckCounter = 0;
            turnTickCount = 0;
            headingFrozen = false;
            System.out.println("============================================");
            System.out.printf("[WANDER] Battery below %.0f%% — leaving CLEAN, all waypoints abandoned.%n",
                    CLEAN_BATT_MIN_PCT);
            System.out.println("[WANDER] Moving randomly until battery critical (≤20%).");
            System.out.println("============================================");
        }

        int nowSec = wanderTimer.getSec();

        if (nextWanderTurnSec < 0) {
            nextWanderTurnSec = nowSec + WANDER_INTERVAL_SEC;
        }

        if (wanderTurnEndSec < 0 && nowSec >= nextWanderTurnSec) {
            wanderSteerCmd = (Math.random() * 2.0 - 1.0) * WANDER_MAX_STEER;
            wanderTurnEndSec = nowSec + WANDER_TURN_DURATION_SEC;
        }

        if (wanderTurnEndSec >= 0 && nowSec >= wanderTurnEndSec) {
            wanderSteerCmd = 0.0;
            wanderTurnEndSec = -1;
            nextWanderTurnSec = nowSec + WANDER_INTERVAL_SEC;
        }

        setVel((float) (DRIVE_SPEED + wanderSteerCmd), (float) (DRIVE_SPEED - wanderSteerCmd));
    }


    /********************************************************************************************************************
     *                                           update() and main()                                                    *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::toggleAutonomousMode()
     * Purpose    : FXML handler for the "Auto Mode" button. Starts or stops autonomous control.
     * Notes      : When enabled, disables GUI teleoperation and init the subsumption controller.
     *              When disabled, re-enables manual GUI control.
     **/
    public void toggleAutonomousMode() {
        if (!autonomousEnabled) {
            // Start autonomous mode:
            autonomousEnabled = true;
            autoInitialized = false;  // trigger init on next run() call
            runMotion = false;          // Stop GUI-driven teleop
            dir = 's';                  // Ensure stop command doesn't bleed through
            if (btnAutoMode != null) {
                btnAutoMode.setText("Auto Mode: ON");
                btnAutoMode.setStyle("-fx-background-color: #7FFF00; ");
            }
            System.out.println("[GUI] Autonomous mode enabled.");
        } else {
            // Stop autonomous mode — re-enable manual control:
            autonomousEnabled = false;
            autoInitialized = false;
            runMotion = true;
            dir = 's';
            setVel(0, 0);
            currentBehavior = "MANUAL";
            if (btnAutoMode != null) {
                btnAutoMode.setText("Auto Mode: OFF");
                btnAutoMode.setStyle("");
            }
            System.out.println("[GUI] Autonomous mode disabled — manual control restored.");
        }
    }

    /**
     * Method     : Controller::update()
     * Purpose    : Called by the update thread on every tick. Runs template matching for camera.
     * Notes      : Template matching updates getTargetX(), getTargetY(), getTargetMaxScore()
     *              which can be used by TRACK behaviour for charger marker detection.
     **/
    public void update() {
        templateMatchingCV(getImage());
    }

    /**
     * Method     : Controller::main()
     * Purpose    : Called by the main thread on every tick. Delegates to run() (subsumption coordinator).
     **/
    public void main() {
        run();
    }


    /********************************************************************************************************************
     *                                     Subsumption Architecture Coordinator                                         *
     *                                                                                                                  *
     *  On every tick:                                                                                                  *
     *    1. Read front sensors (0, 1, 2), battery, GPS position                                                        *
     *    2. Update heading estimate from GPS movement                                                                  *
     *    3. Evaluate TLU for each behaviour (highest priority first):                                                  *
    *         AVOID  → if front obstacle detected                                                                      *
    *         TRACK  → if battery ≤ 20% AND camera marker is detected                                                  *
    *         CLEAN  → if battery ≥ 40%                                                                                *
    *         WANDER → always (fallback, scheduled random manoeuvres)                                                  *
     *    4. Execute the highest-priority behaviour that fires                                                           *
     *    5. Periodically log sensor data for experiment export                                                          *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::run()
     * Purpose    : The main subsumption coordination loop.
     * Notes      : Called at ~1000Hz by the main thread.
     *              First call initialises autonomous mode (disables GUI teleoperation, generates waypoints).
     *              Subsequent calls evaluate and execute behaviours.
     **/
    public void run() {

        // ── AUTONOMOUS MODE NOT YET ENABLED — wait for button press ─────────
        if (!autonomousEnabled) {
            return; // do nothing; GUI teleoperation handles motion
        }

        // ── INITIALISATION (first call after enable) ───────────────────────
        if (!autoInitialized) {
            // Disable GUI teleoperation to prevent it overriding autonomous motor commands
            runMotion = false;
            initCleaningWaypoints();
            experimentStartMs = System.currentTimeMillis();
            autoInitialized = true;

            System.out.println("============================================");
            System.out.println("  AUTONOMOUS MODE STARTED");
            System.out.println("  Room: [" + ROOM_MIN_X + " to " + ROOM_MAX_X + "] x ["
                    + ROOM_MIN_Y + " to " + ROOM_MAX_Y + "]");
            System.out.println("  Charger: (" + CHARGER_XCOORD + ", " + CHARGER_YCOORD + ")");
            System.out.println("  Waypoints: " + cleaningWaypoints.length);
            System.out.println("  Battery: " + getBatteryPercentage() + "% (" + getBatteryTime() + "s)");
            System.out.println("============================================");
            return; // Skip first tick to let sensors stabilise
        }

        // ── DOCKED → handle charging timer here (executeTrack is no longer called) ──
        if (dockedAtCharger) {
            setVel(0, 0);
            if (!chargingComplete) {
                if (chargeStartMs == 0) {
                    chargeStartMs = System.currentTimeMillis();
                    chargeStartBattPct = getBatteryPercentage();
                    batteryTimer.suspend(); // Freeze battery drain while charging
                    chargingOverridePct = chargeStartBattPct;
                    System.out.println("============================================");
                    System.out.println("  DOCKED AT CHARGING STATION");
                    System.out.println("  Battery (before charge): " + String.format("%.1f", chargeStartBattPct) + "%");
                    System.out.println("  Charging for " + (CHARGE_DURATION_MS / 1000) + "s...");
                    System.out.println("============================================");
                }
                long elapsedCharge = System.currentTimeMillis() - chargeStartMs;
                double chargeProgress = Math.min(1.0, (double) elapsedCharge / CHARGE_DURATION_MS);

                // Gradually increase battery from dock-time % to 100%
                chargingOverridePct = chargeStartBattPct + (100.0 - chargeStartBattPct) * chargeProgress;

                if (elapsedCharge >= CHARGE_DURATION_MS) {
                    batteryTimer.restart(); // counter=0 = full battery
                    chargingOverridePct = -1.0; // Clear override
                    chargeStartMs = 0;
                    chargingComplete = true;
                    System.out.println("[CHARGE] Battery restored to 100%. Ending experiment.");
                    if (!experimentExported) {
                        exportExperimentData("DOCKED_AND_CHARGED");
                    }
                }
            }
            return;
        }

        // ── READ SENSORS ──────────────────────────────────────────────────────
        double s0 = getSonarRange(0);       // Front-left  ultrasonic (0-1m)
        double s1 = getSonarRange(1);       // Front-centre ultrasonic
        double s2 = getSonarRange(2);       // Front-right  ultrasonic
        double battPct = getBatteryPercentage();  // Battery % (continuous 0.0–100.0)
        double gpsX = getGPSX();            // Robot X position
        double gpsY = getGPSY();            // Robot Y position

        // ── UPDATE HEADING ESTIMATES ──────────────────────────────────────────
        // 1. Encoder dead-reckoning: uses raw joint angles for proper ±π wrapping
        updateHeadingFromEncoders(currentLeftJointPosition, currentRightJointPosition);
        // 2. GPS heading: supplements encoders for drift correction over distance
        estimateHeadingFromGPS(gpsX, gpsY);

        // ── COVERAGE UPDATE ──────────────────────────────────────────────────
        markCoverage(gpsX, gpsY);

        // ── BATTERY DEAD CHECK (before any behaviour) ─────────────────────
        if (battPct <= 0) {
            setVel(0, 0);  // Stop motors immediately when battery is dead
            if (!experimentExported) {
                System.out.println("[BATTERY] Battery depleted! Stopping motors and exporting data...");
                exportExperimentData("BATTERY_DEAD");
            }
            return;  // Do not execute any behaviour — robot is dead
        }

        // ── AVOID SETTLE: count down heading freeze after AVOID ends ─────────
        // When AVOID is active it sets headingFrozen = true and resets the counter.
        // After AVOID resolves, the counter ticks down. Once it reaches 0, heading
        // estimation resumes. This prevents the corrupted AVOID heading from
        // leaking into CLEAN/TRACK navigation.
        if (avoidSettleCounter > 0 && !shouldAvoid(s0, s1, s2)) {
            avoidSettleCounter--;
            if (avoidSettleCounter == 0) {
                headingFrozen = false;
            }
        }

        // ── SUBSUMPTION: evaluate behaviours in priority order ────────────────
        //    AVOID (1st) > TRACK (2nd) > CLEAN (3rd) > WANDER (4th)

        if (shouldAvoid(s0, s1, s2)) {
            executeAvoid(s0, s1, s2);
        }
        else if (shouldTrack(battPct, getTargetMaxScore())) {
            executeTrack(gpsX, gpsY, getTargetMaxScore());
        }
        else if (shouldClean(battPct)) {
            executeClean(gpsX, gpsY);
        }
        else {
            executeWander();
        }

        // ── DATA LOGGING (every ~250 ticks ≈ every 10 seconds at ~24Hz) ──────
        loopCounter++;
        if (loopCounter % 250 == 0) {
            long elapsedMs = System.currentTimeMillis() - experimentStartMs;
            double elapsedSec = elapsedMs / 1000.0;

                dataLog.add(String.format("%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%s",
                    elapsedSec, gpsX, gpsY, battPct,
                    getLeftWheelEnc(), getRightWheelEnc(),
                    getCoveragePercent(),
                    currentBehavior));
        }

        // ── STATUS PRINT (throttled) ──────────────────────────────────────────
        if (loopCounter % STATUS_PRINT_INTERVAL == 0) {
            System.out.printf("[%s] GPS(%.2f, %.2f) Batt:%.0f%% Hdg:%.0f° Dist:%.1fm Sonar[%.2f %.2f %.2f] WP:%d/%d%n",
                    currentBehavior, gpsX, gpsY, battPct,
                    Math.toDegrees(robotHeading), distanceTravelled,
                    s0, s1, s2,
                    totalWaypointsReached, cleaningWaypoints != null ? cleaningWaypoints.length : 0);
        }
    }


    /********************************************************************************************************************
     *                                         Experiment Data Export                                                    *
     *                                                                                                                  *
     *  Exports collected sensor data to CSV files for analysis:                                                        *
     *    1. GPS trajectory data (time, x, y, battery, wheel encoders, behaviour)                                       *
     *    2. Summary report (total distance, behaviour activations, outcome)                                            *
     ********************************************************************************************************************/

    /**
     * Method     : Controller::exportExperimentData()
     * Purpose    : Writes collected experiment data to CSV and summary files.
     * Parameters : reason - why the experiment ended ("DOCKED", "BATTERY_DEAD", etc.)
     **/
    private void exportExperimentData(String reason) {
        if (experimentExported) return;
        experimentExported = true;

        long timestamp = System.currentTimeMillis();
        long totalTimeMs = timestamp - experimentStartMs;
        double totalTimeSec = totalTimeMs / 1000.0;

        // ── Export GPS trajectory CSV ─────────────────────────────────────
        String trajectoryFile = "gps_trajectory_" + timestamp + ".csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(trajectoryFile))) {
            pw.println("time_sec,gps_x,gps_y,battery_pct,left_enc,right_enc,coverage_pct,behavior");
            for (String row : dataLog) {
                pw.println(row);
            }
            System.out.println("[EXPORT] Trajectory saved: " + trajectoryFile + " (" + dataLog.size() + " rows)");
        } catch (Exception e) {
            System.err.println("[EXPORT] Error writing trajectory: " + e.getMessage());
        }

        // ── Export summary report ─────────────────────────────────────────
        String summaryFile = "experiment_report_" + timestamp + ".txt";
        try (PrintWriter pw = new PrintWriter(new FileWriter(summaryFile))) {
            pw.println("=== EXPERIMENT SUMMARY ===");
            pw.println("End reason       : " + reason);
            pw.println("Total time       : " + String.format("%.1f", totalTimeSec) + " seconds");
            pw.println("Distance travelled: " + String.format("%.2f", distanceTravelled) + " metres");
            pw.println("Coverage: " + String.format("%.1f", getCoveragePercent()) + "% of room area");
            pw.println("Final GPS        : (" + getGPSX() + ", " + getGPSY() + ")");
            pw.println("Final battery    : " + getBatteryPercentage() + "%");
            int patternPasses = totalWaypointsReached / cleaningWaypoints.length;
            int partialWPs = totalWaypointsReached % cleaningWaypoints.length;
            pw.println("Waypoints reached: " + totalWaypointsReached
                    + " (" + patternPasses + " full passes + " + partialWPs + " partial)");
            pw.println("");
            pw.println("Behaviour activations (ticks):");
            pw.println("  AVOID  : " + avoidActivations);
            pw.println("  TRACK  : " + trackActivations);
            pw.println("  CLEAN  : " + cleanActivations);
            pw.println("  WANDER : " + wanderActivations);
            pw.println("");
            pw.println("Charger location : (" + CHARGER_XCOORD + ", " + CHARGER_YCOORD + ")");
            pw.println("Room boundaries  : [" + ROOM_MIN_X + ", " + ROOM_MAX_X + "] x ["
                    + ROOM_MIN_Y + ", " + ROOM_MAX_Y + "]");
            pw.println("Cleaning area    : [" + (ROOM_MIN_X+WALL_MARGIN) + ", " + (ROOM_MAX_X-WALL_MARGIN) + "] x ["
                    + (ROOM_MIN_Y+WALL_MARGIN) + ", " + (ROOM_MAX_Y-WALL_MARGIN) + "]");
            pw.println("Wall margin      : " + WALL_MARGIN + " metres");
            pw.println("Lane spacing     : " + LANE_SPACING + " metres");
            int totalTicks = avoidActivations + trackActivations + cleanActivations + wanderActivations;
            pw.println("AVOID fraction   : " + (totalTicks>0 ? String.format("%.1f", 100.0*avoidActivations/totalTicks) : "0") + "%");
            pw.println("Docked           : " + dockedAtCharger);
            System.out.println("[EXPORT] Summary saved: " + summaryFile);
        } catch (Exception e) {
            System.err.println("[EXPORT] Error writing summary: " + e.getMessage());
        }

        // ── Export coverage grid ────────────────────────────────────────────
        String coverageFile = "coverage_grid_" + timestamp + ".csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(coverageFile))) {
            pw.println("# Coverage grid: " + coverageRows + "x" + coverageCols
                    + "  cell_m=" + String.format("%.3f", COVERAGE_RES)
                    + "  origin=(" + CLEAN_MIN_X + "," + CLEAN_MIN_Y + ")");
            for (int r = 0; r < coverageRows; r++) {
                for (int c = 0; c < coverageCols; c++) {
                    pw.print(coverageGrid[r][c] ? "1" : "0");
                    if (c < coverageCols - 1) pw.print(",");
                }
                pw.println();
            }
            System.out.println("[EXPORT] Coverage grid saved: " + coverageFile + " (" + coverageRows + "x" + coverageCols + ")");
        } catch (Exception e) {
            System.err.println("[EXPORT] Error writing coverage grid: " + e.getMessage());
        }

        // ── Print summary to console ──────────────────────────────────────
        System.out.println("============================================");
        System.out.println("  EXPERIMENT COMPLETE: " + reason);
        System.out.println("  Time: " + String.format("%.1f", totalTimeSec) + "s");
        System.out.println("  Distance: " + String.format("%.2f", distanceTravelled) + "m");
        System.out.println("  Coverage: " + String.format("%.1f", getCoveragePercent()) + "%");
        System.out.println("  Waypoints: " + totalWaypointsReached + " total ("
                + (totalWaypointsReached / cleaningWaypoints.length) + " full passes)");
        System.out.println("  Docked: " + dockedAtCharger);
        System.out.println("============================================");
    }
}

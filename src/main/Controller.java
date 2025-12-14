package main;

import coppelia.*;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.javacpp.indexer.UByteIndexer;
import org.bytedeco.javacv.Java2DFrameConverter;
import org.bytedeco.javacv.OpenCVFrameConverter;

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
import java.util.Collections;

/**
 * Created by Theo Theodoridis.
 * Class    : Controller
 * Version  : v1.0
 * Date     : © Copyright 01/11/2020
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
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
    @FXML
    private Button btnAutoMode;
    @FXML
    private Label lblBehavior;
    @FXML
    private Label lblBattery;
    @FXML
    private Label lblBatteryTime;
    @FXML
    private Label lblCharging;

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
     * Subsumption Architecture & Behaviors:
     **/
    private boolean autonomousMode = false;
    private String currentBehavior = "IDLE";
    
    // Room boundaries
    private static final double ROOM_MAX_X = 2.15;
    private static final double ROOM_MIN_X = -2.15;
    private static final double ROOM_MAX_Y = 2.15;
    private static final double ROOM_MIN_Y = -2.15;
    
    // Clean behavior state
    private boolean cleaningActive = false;
    private double cleanTargetX = ROOM_MIN_X;
    private double cleanTargetY = ROOM_MIN_Y;
    private double cleanStripWidth = 0.3; // Width of cleaning strip
    private boolean cleanGoingRight = true;
    
    // TLU thresholds
    private static final double OBSTACLE_THRESHOLD = 0.5; // meters
    private static final double TARGET_DETECT_THRESHOLD = 0.7; // Template matching score
    private static final double BATTERY_LOW_THRESHOLD = 20.0; // percentage
    private static final double BATTERY_CRITICAL_THRESHOLD = 10.0; // percentage
    
    // Wander behavior state
    private long lastWanderChange = 0;
    private int wanderDirection = 0;

    /**
     * Camera:
     **/

    private double targetMinScore = 0.0;
    private double targetMaxScore = 0.0;

    private int resolutionCamera = 256;
    private org.bytedeco.opencv.opencv_core.Point point = new org.bytedeco.opencv.opencv_core.Point();
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

    private int MAX_BATT_TIME = 60 * 20; // Default 20 mins battery time.
    private final int MAX_BATT_VOLT = 12;      // volts.

    private Timer motionTimer = new Timer();
    private Timer batteryTimer = new Timer();
    
    // Charging state
    private boolean isCharging = false;
    private long chargingStartTime = 0;
    private static final double CHARGING_DISTANCE = 0.3; // Distance to charger to start charging
    private static final int CHARGE_RATE_SEC = 2; // Seconds to charge per 1% battery


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
    }

    public double getBatteryCapacity() {
        double v = (double) MAX_BATT_VOLT - Utils.map(batteryTimer.getSec(), 0, (double) MAX_BATT_TIME, 0, (double) MAX_BATT_VOLT);
        return ((v > 0.0) ? v : 0.0);
    }

    public double getBatteryPercentage() {
        double v = getBatteryCapacity();

        if ((v >= 9.6) && (v <= 12)) return (100.0);
        else if ((v >= 7.2) && (v < 9.6)) return (80.0);
        else if ((v >= 4.8) && (v < 7.2)) return (60.0);
        else if ((v >= 2.4) && (v < 4.8)) return (40.0);
        else if ((v > 1.0) && (v < 2.4)) return (20.0);
        else
            return (0.0);
    }

    public boolean getBatteryState() {
        double v = getBatteryCapacity();
        return ((v > 0.0) ? true : false);
    }
    
    /**
     * Check if robot is at charging station and handle charging
     */
    private void checkAndHandleCharging() {
        double currentX = getGPSX();
        double currentY = getGPSY();
        
        // Calculate distance to charger
        double dx = CHARGER_XCOORD - currentX;
        double dy = CHARGER_YCOORD - currentY;
        double distanceToCharger = Math.sqrt(dx*dx + dy*dy);
        
        // Check if at charging station
        if (distanceToCharger < CHARGING_DISTANCE) {
            if (!isCharging && getBatteryPercentage() < 100.0) {
                // Start charging
                isCharging = true;
                chargingStartTime = System.currentTimeMillis();
                setVel(0, 0); // Stop robot while charging
                System.out.println("CHARGING STARTED at " + getBatteryPercentage() + "%");
            }
            
            if (isCharging) {
                // Charging logic - restore battery
                long chargingDuration = (System.currentTimeMillis() - chargingStartTime) / 1000; // seconds
                int percentageToRestore = (int)(chargingDuration / CHARGE_RATE_SEC);
                
                if (percentageToRestore > 0) {
                    // Reduce battery timer to increase capacity
                    int currentTime = batteryTimer.getSec();
                    int timeToReduce = (int)((percentageToRestore / 100.0) * MAX_BATT_TIME);
                    int newTime = Math.max(0, currentTime - timeToReduce);
                    batteryTimer.setSec(newTime);
                }
                
                // Check if fully charged
                if (getBatteryPercentage() >= 100.0) {
                    isCharging = false;
                    cleaningActive = true; // Resume cleaning after charging
                    System.out.println("CHARGING COMPLETE - Battery at 100%");
                }
            }
        } else {
            if (isCharging) {
                isCharging = false;
                System.out.println("CHARGING STOPPED - Moved away from charger");
            }
        }
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
        // [1]Get image dimensions:
        int imageType = -1;
        int w = bimg.getWidth();
        int h = bimg.getHeight();

        // [2]Select image type: color/grayscale
        if (color) imageType = bimg.getType();
        else imageType = BufferedImage.TYPE_BYTE_GRAY;

        // [3]Rotate and draw:
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
        // [1]Load source and template image files:
        Java2DFrameConverter converter = new Java2DFrameConverter();
        OpenCVFrameConverter.ToMat cvConverter = new OpenCVFrameConverter.ToMat();
        
        Mat src = cvConverter.convert(converter.convert(image));
        Mat srcGray = new Mat();
        
        // Check if image is already grayscale or convert from BGR/RGB
        if (src.channels() == 1) {
            srcGray = src.clone();
        } else if (src.channels() == 3) {
            opencv_imgproc.cvtColor(src, srcGray, opencv_imgproc.COLOR_BGR2GRAY);
        } else if (src.channels() == 4) {
            opencv_imgproc.cvtColor(src, srcGray, opencv_imgproc.COLOR_BGRA2GRAY);
        }
        
        Mat tmp = opencv_imgcodecs.imread("data/images/marker.jpg", opencv_imgcodecs.IMREAD_GRAYSCALE);

        // [2]The Correlation Image Result:
        int resultCols = srcGray.cols() - tmp.cols() + 1;
        int resultRows = srcGray.rows() - tmp.rows() + 1;
        Mat result = new Mat(resultRows, resultCols, opencv_core.CV_32FC1);

        // [3]Select a function template-match method:
        opencv_imgproc.matchTemplate(srcGray, tmp, result, opencv_imgproc.TM_CCOEFF_NORMED);

        double[] minVal = new double[1];
        double[] maxVal = new double[1];
        org.bytedeco.opencv.opencv_core.Point minLoc = new org.bytedeco.opencv.opencv_core.Point();
        org.bytedeco.opencv.opencv_core.Point maxLoc = new org.bytedeco.opencv.opencv_core.Point();

        // [4]Compute and print min-max value locations:
        opencv_core.minMaxLoc(result, minVal, maxVal, minLoc, maxLoc, null);
        targetMinScore = minVal[0]; // Min Score.
        targetMaxScore = maxVal[0]; // Max Score.
        //System.out.println("Min: " + targetMinScore);
        //System.out.println("Max: " + targetMaxScore);

        // [5]Mark at point the image template coordinates:
        point.x(maxLoc.x() + tmp.cols());
        point.y(maxLoc.y() + tmp.rows());

        // [6]Draw the rectangle result in source image:
        org.bytedeco.opencv.opencv_core.Point bottomRight = new org.bytedeco.opencv.opencv_core.Point(maxLoc.x() + tmp.cols(), maxLoc.y() + tmp.rows());
        opencv_imgproc.rectangle(src, maxLoc, bottomRight, new Scalar(128, 128, 128, 0), 2, 8, 0);

        // [7]Display the image:
        BufferedImage resultImage = converter.convert(cvConverter.convert(src));
        ImageViewer.display(resultImage);
        
        // Clean up
        src.release();
        srcGray.release();
        tmp.release();
        result.release();
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
    
    @FXML
    public void toggleAutonomousMode() {
        autonomousMode = !autonomousMode;
        if (autonomousMode) {
            btnAutoMode.setText("Auto Mode: ON");
            btnAutoMode.setStyle("-fx-background-color: #7FFF00; ");
            setAutonomousMode(true);
            cleaningActive = true; // Start cleaning when autonomous mode enabled
        } else {
            btnAutoMode.setText("Auto Mode: OFF");
            btnAutoMode.setStyle("");
            setAutonomousMode(false);
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
        motionTimer.start();
        batteryTimer.start();

        update.start();
        main.start();
    }

    /**
     * Method     : Controller::update()
     * Purpose    : To update the robot.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    private Thread update = new Thread() {
        public void run() {
            setBatteryTime(20);
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
                Platform.runLater(new Runnable() {
                    public void run() {
                        if (runGPS) {
                            lblGpsX.setText("X: " + gpsValues[0]);
                            lblGpsY.setText("Y: " + gpsValues[1]);
                            lblGpsZ.setText("Z: " + gpsValues[2]);
                        }
                        if (runSensors) {
                            lblSensor0.setText(sonarValues[0] + "m");
                            lblSensor1.setText(sonarValues[1] + "m");
                            lblSensor2.setText(sonarValues[2] + "m");
                            lblSensor3.setText(sonarValues[3] + "m");
                            lblSensor4.setText(sonarValues[4] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                        }
                        if (runWheelEncoder) {
                            lblRightWheel.setText("Right : " + encoderValues[0]);
                            lblLeftWheel.setText("Left : " + encoderValues[1]);
                        }
                        
                        // Update behavior display
                        if (autonomousMode) {
                            lblBehavior.setText("Behavior: " + currentBehavior);
                            lblBehavior.setStyle("-fx-text-fill: #00FF00;");
                        } else {
                            lblBehavior.setText("MANUAL MODE");
                            lblBehavior.setStyle("-fx-text-fill: #FFFF00;");
                        }
                        
                        // Update battery display
                        double batteryPct = getBatteryPercentage();
                        lblBattery.setText(String.format("Battery: %.0f%%", batteryPct));
                        lblBatteryTime.setText("Time: " + getBatteryTime() + "s");
                        
                        // Color code battery level
                        if (batteryPct >= 60) {
                            lblBattery.setStyle("-fx-text-fill: #00FF00;"); // Green
                        } else if (batteryPct >= 20) {
                            lblBattery.setStyle("-fx-text-fill: #FFAA00;"); // Orange
                        } else {
                            lblBattery.setStyle("-fx-text-fill: #FF0000;"); // Red
                        }
                        
                        // Update charging status
                        if (isCharging) {
                            lblCharging.setText("⚡ CHARGING");
                            lblCharging.setStyle("-fx-text-fill: #00FFFF;");
                        } else {
                            lblCharging.setText("");
                        }
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
     * Purpose    : To run the main code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
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
     *                                                   Student Code                                                   *
     ********************************************************************************************************************/

//    private FSM avoid = new Avoid(3, 100);
//    private FSM track  = new Track( 75, 3);
//    private FSM clean  = new Clean( 50, 3);
//    private FSM wander = new Wander(3, 25);

    /**
     * Method     : Controller::update()
     * Purpose    : To update custom code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : Comment where appropriate.
     **/
    public void update() {
        // Check and handle charging
        checkAndHandleCharging();
        
        // Autonomous mode with Subsumption Architecture
        if (autonomousMode && !isCharging) {
            subsumptionControl();
        } else if (isCharging) {
            // Robot is charging - keep stopped
            setVel(0, 0);
        }
        
        // Template matching for target detection (used by Track behavior)
        // Only run if battery is low and we need to track charger
        if (getBatteryPercentage() < BATTERY_LOW_THRESHOLD && !isCharging) {
            try {
                templateMatchingCV(getImage());
            } catch (Exception e) {
                // Suppress template matching errors if marker not found
            }
        }

        // Print sensors for debugging (uncomment if needed)
//        System.out.println("\nWheel(R): " + getRightWheelEnc() + ", Wheel(L): " + getLeftWheelEnc());
//        System.out.println("GPS(X): " + getGPSX() + ", GPS(Y): " + getGPSY() + ", GPS(Z): " + getGPSZ());
//        System.out.println("C: " + getBatteryCapacity() + "v, P: " + getBatteryPercentage() + "%, S: " + getBatteryState() + ", T: " + getBatteryTime() + "sec");
//        for(int i=0 ; i<getSonarNo() ; i++) System.out.println(i + ": " + Utils.getDecimal(getSonarRange(i), "0.0"));
    }

    /**
     * Subsumption Architecture Implementation
     * Priority: Avoid > Track > Clean > Wander
     */
    private void subsumptionControl() {
        // PRIORITY 1: AVOID - Obstacle avoidance (highest priority)
        if (shouldAvoid()) {
            currentBehavior = "AVOID";
            avoidBehavior();
            return;
        }
        
        // PRIORITY 2: TRACK - Find charging station when battery low
        if (shouldTrack()) {
            currentBehavior = "TRACK";
            trackBehavior();
            return;
        }
        
        // PRIORITY 3: CLEAN - Coverage cleaning pattern
        if (shouldClean()) {
            currentBehavior = "CLEAN";
            cleanBehavior();
            return;
        }
        
        // PRIORITY 4: WANDER - Random exploration (lowest priority)
        currentBehavior = "WANDER";
        wanderBehavior();
    }

    // ========== BEHAVIOR CONDITION CHECKS (TLU Logic) ==========
    
    private boolean shouldAvoid() {
        // Check if any ultrasonic sensor detects obstacle within threshold
        for (int i = 0; i < getSonarNo(); i++) {
            double range = getSonarRange(i);
            if (range > 0 && range < OBSTACLE_THRESHOLD) {
                return true;
            }
        }
        return false;
    }
    
    private boolean shouldTrack() {
        // Track charger when battery is low or critical
        double batteryPct = getBatteryPercentage();
        return batteryPct < BATTERY_LOW_THRESHOLD;
    }
    
    private boolean shouldClean() {
        // Clean when battery is sufficient and not tracking
        double batteryPct = getBatteryPercentage();
        return batteryPct >= BATTERY_LOW_THRESHOLD && cleaningActive;
    }

    // ========== BEHAVIOR IMPLEMENTATIONS ==========
    
    /**
     * AVOID BEHAVIOR: Obstacle avoidance using ultrasonic sensors
     * Strategy: Turn away from obstacles detected by sensors
     */
    private void avoidBehavior() {
        double[] sensors = new double[6];
        for (int i = 0; i < getSonarNo(); i++) {
            sensors[i] = getSonarRange(i);
        }
        
        // Normalize sensor values (closer = higher value)
        double[] normalized = new double[6];
        for (int i = 0; i < 6; i++) {
            if (sensors[i] > 0 && sensors[i] < OBSTACLE_THRESHOLD) {
                normalized[i] = 1.0 - (sensors[i] / OBSTACLE_THRESHOLD);
            } else {
                normalized[i] = 0.0;
            }
        }
        
        // Front sensors (0,1,2) - if obstacle ahead, turn
        double frontLeft = (normalized[0] + normalized[1]) / 2.0;
        double frontRight = (normalized[1] + normalized[2]) / 2.0;
        
        // Side sensors
        double left = normalized[5];
        double right = normalized[3];
        
        // Decision logic
        if (frontLeft > 0.5 && frontRight > 0.5) {
            // Obstacle directly ahead - turn right (or left based on side clearance)
            if (left < right) {
                setVel(-2.0f, 2.0f); // Turn left
            } else {
                setVel(2.0f, -2.0f); // Turn right
            }
        } else if (frontLeft > 0.5) {
            // Obstacle on front-left - turn right
            setVel(3.0f, -1.0f);
        } else if (frontRight > 0.5) {
            // Obstacle on front-right - turn left
            setVel(-1.0f, 3.0f);
        } else if (left > 0.5) {
            // Obstacle on left side - veer right
            setVel(4.0f, 2.0f);
        } else if (right > 0.5) {
            // Obstacle on right side - veer left
            setVel(2.0f, 4.0f);
        } else {
            // Small obstacle - slow down
            setVel(2.0f, 2.0f);
        }
    }
    
    /**
     * TRACK BEHAVIOR: Navigate to charging station using camera and GPS
     * Strategy: Use template matching to find charger, navigate using GPS
     */
    private void trackBehavior() {
        double currentX = getGPSX();
        double currentY = getGPSY();
        
        // Calculate direction to charger
        double dx = CHARGER_XCOORD - currentX;
        double dy = CHARGER_YCOORD - currentY;
        double distance = Math.sqrt(dx*dx + dy*dy);
        
        // If very close to charger, stop
        if (distance < 0.2) {
            setVel(0, 0);
            cleaningActive = false; // Stop cleaning when at charger
            return;
        }
        
        // Use template matching score to adjust navigation
        double targetScore = getTargetMaxScore();
        
        if (targetScore > TARGET_DETECT_THRESHOLD) {
            // Target visible in camera - use visual servoing
            int targetX = getTargetX();
            int imageCenterX = resolutionCamera / 2;
            
            double error = targetX - imageCenterX;
            double turnFactor = error / (resolutionCamera / 2.0);
            
            // Move forward while centering target
            float baseSpeed = 3.0f;
            setVel((float)(baseSpeed - turnFactor * 2.0), 
                   (float)(baseSpeed + turnFactor * 2.0));
        } else {
            // Target not visible - navigate using GPS
            double angle = Math.atan2(dy, dx);
            
            // Simple proportional control
            float speed = 3.0f;
            float turnSpeed = (float)(angle * 1.5);
            
            setVel(speed - turnSpeed, speed + turnSpeed);
        }
    }
    
    /**
     * CLEAN BEHAVIOR: Systematic coverage using boustrophedon (lawnmower) pattern
     * Strategy: Move in parallel strips across the room
     */
    private void cleanBehavior() {
        double currentX = getGPSX();
        double currentY = getGPSY();
        
        // Check if reached target position
        double distToTarget = Math.sqrt(
            Math.pow(cleanTargetX - currentX, 2) + 
            Math.pow(cleanTargetY - currentY, 2)
        );
        
        if (distToTarget < 0.15) {
            // Reached target - plan next strip
            if (cleanGoingRight) {
                cleanTargetY += cleanStripWidth;
                if (cleanTargetY >= ROOM_MAX_Y) {
                    cleanTargetY = ROOM_MAX_Y;
                    cleanTargetX = ROOM_MAX_X;
                    cleanGoingRight = false;
                } else {
                    cleanTargetX = ROOM_MAX_X;
                }
            } else {
                cleanTargetY += cleanStripWidth;
                if (cleanTargetY >= ROOM_MAX_Y) {
                    // Finished cleaning - reset
                    cleanTargetX = ROOM_MIN_X;
                    cleanTargetY = ROOM_MIN_Y;
                    cleanGoingRight = true;
                    cleaningActive = false;
                } else {
                    cleanTargetX = ROOM_MIN_X;
                }
            }
        }
        
        // Navigate to target position
        double dx = cleanTargetX - currentX;
        double dy = cleanTargetY - currentY;
        double angle = Math.atan2(dy, dx);
        
        float speed = 4.0f;
        float turnSpeed = (float)(angle * 2.0);
        
        setVel(speed - turnSpeed, speed + turnSpeed);
    }
    
    /**
     * WANDER BEHAVIOR: Random exploration
     * Strategy: Move forward with occasional random turns
     */
    private void wanderBehavior() {
        long currentTime = System.currentTimeMillis();
        
        // Change direction every 3-5 seconds
        if (currentTime - lastWanderChange > 3000 + Math.random() * 2000) {
            wanderDirection = (int)(Math.random() * 5);
            lastWanderChange = currentTime;
        }
        
        switch (wanderDirection) {
            case 0: // Forward
                setVel(4.0f, 4.0f);
                break;
            case 1: // Forward-right curve
                setVel(4.0f, 3.0f);
                break;
            case 2: // Forward-left curve
                setVel(3.0f, 4.0f);
                break;
            case 3: // Spin right
                setVel(3.0f, -3.0f);
                break;
            case 4: // Spin left
                setVel(-3.0f, 3.0f);
                break;
        }
    }
    
    // Helper methods for behaviors
    public void startCleaning() {
        cleaningActive = true;
        cleanTargetX = ROOM_MIN_X;
        cleanTargetY = ROOM_MIN_Y;
        cleanGoingRight = true;
    }
    
    public void stopCleaning() {
        cleaningActive = false;
    }
    
    public void setAutonomousMode(boolean enabled) {
        autonomousMode = enabled;
        if (enabled) {
            System.out.println("Autonomous mode ENABLED - Subsumption architecture active");
        } else {
            System.out.println("Autonomous mode DISABLED - Manual control");
            setVel(0, 0); // Stop robot
        }
    }
    
    public String getCurrentBehavior() {
        return currentBehavior;
    }

    /**
     * Method     : Controller::main()
     * Purpose    : To run the main code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    public void main()
    {
        run();
    }

    /**
     * Method     : SubsumptionCoordinator::run()
     * Purpose    : To run a custom Subsumption Architecture.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    public void run()
    {
        Integer priority[] = new Integer[2];
        //        int priority[] = new int[4];

        double cam = getTargetMaxScore();                                                      // Target horizontal detection (pixels).
        double bat = getBatteryCapacity();                                                     // Battery capacity (volts).
        double snr = Arrays.stream(getSonarRanges()).min().getAsDouble();                      // Min sonar range radius (meters).
        double gps = Utils.getEuclidean(CHARGER_XCOORD, getGPSY(), getGPSX(), CHARGER_YCOORD); // GPS distance from charger (meters?).
        double sensors[] = new double[]{bat, snr, cam, gps};                                   // Sensor vector.

        double sonarData[] = new double[]
                {
                        getSonarRange(0),
                        getSonarRange(1),
                        getSonarRange(2),
                        getSonarRange(3),
                        getSonarRange(4),
                        getSonarRange(5)
                };

        avoid();
    }

    public void avoid()
    {
        double leftMinSonarRadius  = Arrays.stream(new double[]{getSonarRange(0), getSonarRange(1), getSonarRange(2)}).min().getAsDouble();
        double rightMinSonarRadius = Arrays.stream(new double[]{getSonarRange(3), getSonarRange(4), getSonarRange(5)}).min().getAsDouble();

        if(leftMinSonarRadius < rightMinSonarRadius)
        {
            turnSpot(vel/2, 1000);
        }
        else
        if(leftMinSonarRadius > rightMinSonarRadius)
        {
            turnSpot(-vel/2, 1000);
        }
    }

    /**
     * Method     : Controller::tlu()
     * Purpose    : To implement a Threshold Logic Unit.
     * Parameters : - w_vec : The weight vector.
     * - w_vec : The sensor vector.
     * - f     : The activation threshold.
     * Returns    : True (+1) if TLU is activated, False (-1) otherwise.
     * Notes      : None.
     **/
    public boolean tlu(double w_vec[], double s_vec[], double f)
    {
        return (false);
    }
}

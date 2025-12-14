# 🤖 V-Rep v5 Robot Simulator - Complete Guide

## 📋 TABLE OF CONTENTS
1. [What This Code Does](#what-this-code-does)
2. [Quick Start](#quick-start)
3. [Detailed Code Analysis](#detailed-code-analysis)
4. [Running Instructions](#running-instructions)
5. [Troubleshooting](#troubleshooting)

---

## 🎯 WHAT THIS CODE DOES

This is a **Roomba vacuum robot simulator** with full JavaFX GUI that interfaces with **CoppeliaSim (V-REP)** for realistic 3D physics simulation.

### ✅ APIs Integrated:
- **CoppeliaSim Remote API** ✅ - Full integration via `remoteApi.java`
- **JavaFX** ✅ - Modern GUI framework
- **JavaCV/OpenCV** ✅ - Computer vision for template matching

### 🔧 Implemented Features:

#### **Sensors:**
- ✅ 6x Proximity/Ultrasonic sensors (obstacle detection)
- ✅ GPS system (X, Y, Z positioning)
- ✅ Vision camera (256x256 resolution)
- ✅ Wheel encoders (odometry tracking)
- ✅ Battery monitor (voltage-based, 20min default)

#### **Control:**
- ✅ Manual teleoperation (GUI buttons)
- ✅ Autonomous obstacle avoidance
- ✅ Spot turning & smooth turning
- ✅ Speed control (configurable velocity)

#### **Vision:**
- ✅ OpenCV template matching (finds markers in camera view)
- ✅ Real-time camera feed display
- ✅ Target scoring and localization

#### **Architecture:**
- ✅ Multi-threaded updates (non-blocking GUI)
- ✅ Finite State Machine interface (for behavior-based AI)
- ✅ Battery management system
- ✅ Real-time sensor data visualization

---

## 🚀 QUICK START

### Prerequisites:
1. **CoppeliaSim** (V-REP) installed
2. **Java 8+** (Java 8 preferred for built-in JavaFX)
3. **Linux** native libraries for Remote API

### 3-Step Launch:

```bash
# Step 1: Run setup (installs dependencies)
cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
./setup.sh

# Step 2: Start CoppeliaSim and load a scene
# Open one of: scenes/room_top_empty.ttt, room_top_full.ttt, or room_top_part.ttt

# Step 3: Compile and run
./compile_and_run.sh
```

---

## 🔍 DETAILED CODE ANALYSIS

### File Structure:

```
V-Rep v5/
├── src/
│   ├── coppelia/          # CoppeliaSim Remote API wrapper classes
│   │   ├── remoteApi.java    # Main Remote API interface
│   │   ├── IntW.java         # Integer wrapper for API calls
│   │   ├── FloatW.java       # Float wrapper
│   │   ├── BoolW.java        # Boolean wrapper
│   │   └── [other wrappers]  # CharW, StringW, Array wrappers
│   │
│   ├── main/              # Core application
│   │   ├── Main.java         # JavaFX launcher (22 lines)
│   │   ├── Controller.java   # MAIN CONTROLLER (905 lines) ⭐
│   │   ├── FSM.java          # Finite State Machine interface
│   │   ├── Entry.java        # FSM data entry class
│   │   └── GUI.fxml          # JavaFX UI layout
│   │
│   └── utils/             # Utility classes
│       ├── Timer.java        # Timing utilities
│       ├── Delay.java        # Sleep/delay functions
│       ├── Utils.java        # Math utilities (Euclidean distance, mapping)
│       ├── ImageViewer.java  # Image display
│       └── Rand.java         # Random number generation
│
├── lib/javacv/            # Computer vision libraries
│   ├── javacv-0.1.jar
│   ├── javacpp-0.1.jar
│   └── OpenCV.jar
│
├── scenes/                # CoppeliaSim scene files
│   ├── room_top_empty.ttt    # Empty test room
│   ├── room_top_full.ttt     # Room with obstacles
│   └── room_top_part.ttt     # Partially filled room
│
├── data/images/           # Template images for matching
│
└── [native libraries]     # .dll (Windows) - need .so for Linux
```

---

### 🧠 Controller.java - The Brain (905 lines)

This is the **main controller** that orchestrates everything:

#### **Connection & Setup:**
```java
Line 161: private remoteApi vRep = new remoteApi();
Line 770: clientID = vRep.simxStart("127.0.0.1", 20001, true, true, 5000, 5);
```
- Connects to CoppeliaSim on localhost:20001
- 5-second timeout for connection

#### **Sensor Reading Methods:**

**GPS (Lines 305-325):**
```java
public double[] readGPS()
```
- Reads robot position from CoppeliaSim
- Returns [X, Y, Z] coordinates
- Rounds to 2 decimal places

**Ultrasonic Sensors (Lines 337-445):**
```java
private double readSonarRange(int sensor)
```
- Reads 6 proximity sensors (Proximity_sensor0-5)
- Returns distance 0.0-1.0 meters
- Uses Euclidean distance calculation

**Camera (Lines 460-490):**
```java
private Color[][] readCamera()
```
- Captures 256x256 image from Vision_sensor
- Converts to Color matrix
- Supports template matching with OpenCV

**Wheel Encoders (Lines 224-281):**
```java
private double readRightWheelEnc()
private double readLeftWheelEnc()
```
- Tracks wheel rotations
- Calculates angular displacement
- Returns total revolutions

#### **Motion Control Methods:**

**Basic Movement (Lines 630-680):**
```java
public void move(float vel)          // Forward/backward
public void turnSpot(float vel)      // Rotate in place
public void turnSharp(float vel)     // Sharp turn (one wheel stopped)
public void turnSmooth(float vel)    // Smooth arc turn
```

**Timed Movement:**
```java
public void move(float vel, int time)     // Move for X milliseconds
public void turnSpot(float vel, int time) // Turn for X milliseconds
```

**Teleoperation (Lines 722-740):**
```java
public void teleoperate(char dir, int vel)
```
- 's' = stop
- 'f' = forward
- 'b' = backward
- 'r' = turn right
- 'l' = turn left

#### **Vision Processing (Lines 540-595):**

**Template Matching:**
```java
public void templateMatchingCV(BufferedImage image)
```
- Uses OpenCV `CV_TM_CCOEFF_NORMED` algorithm
- Searches for marker in `data/images/marker.jpg`
- Returns confidence score (0-1)
- Marks location with rectangle

#### **Battery System (Lines 177-219):**
```java
public double getBatteryCapacity()    // Returns voltage (0-12V)
public double getBatteryPercentage()  // Returns 0-100%
public boolean getBatteryState()      // Returns true if alive
```
- 12V max capacity
- Drains over 20 minutes (configurable)
- Robot stops when depleted

#### **Obstacle Avoidance (Lines 883-900):**
```java
public void avoid()
```
- Compares left sensors (0,1,2) vs right sensors (3,4,5)
- Turns away from closer obstacle
- Simple reactive behavior

#### **Multi-threading (Lines 800-870):**

**Update Thread:**
- Reads all sensors continuously
- Updates GUI labels (GPS, sonars, encoders)
- Runs motion commands
- Checks battery status
- 1ms cycle time

**Main Thread:**
- Runs user code (`main()` method)
- Calls obstacle avoidance
- Template matching
- Custom behaviors

---

### 🎮 GUI.fxml - User Interface

**Controls:**
- Connect button → Establishes CoppeliaSim connection
- Direction buttons → Manual control (↑ ↓ ← → ⏹)
- Labels → Real-time sensor display

**Display Sections:**
- Camera feed (256x256 canvas)
- GPS coordinates (X, Y, Z)
- 6 Sonar readings
- Wheel encoder values
- Battery status

---

### 🔌 Remote API Integration

The CoppeliaSim Remote API is fully integrated through wrapper classes:

**Key API Calls Used:**
```java
simxStart()                    // Connect to simulator
simxGetObjectHandle()          // Get robot/sensor handles
simxGetObjectPosition()        // Read GPS
simxReadProximitySensor()      // Read ultrasonic
simxGetVisionSensorImage()     // Read camera
simxGetJointPosition()         // Read encoders
simxSetJointTargetVelocity()   // Control motors
simxFinish()                   // Disconnect
```

**Communication Modes:**
- `simx_opmode_streaming` → Start continuous data stream
- `simx_opmode_buffer` → Read from buffer (fast)
- `simx_opmode_blocking` → Wait for response
- `simx_opmode_oneshot` → Single command

---

## 🏃 RUNNING INSTRUCTIONS

### Method 1: Automated (Recommended)

```bash
cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"

# First time setup
./setup.sh

# Every time you want to run
./compile_and_run.sh
```

### Method 2: Manual Compilation

```bash
cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"

# Set classpath
CLASSPATH="lib/javacv/javacv-0.1.jar:lib/javacv/javacpp-0.1.jar:lib/javacv/OpenCV.jar"

# Compile
javac -d "out/production/V-Rep v5" \
      -cp "$CLASSPATH" \
      -sourcepath src \
      src/coppelia/*.java \
      src/utils/*.java \
      src/main/*.java

# Copy resources
cp src/main/GUI.fxml "out/production/V-Rep v5/main/"

# Run
java -cp "out/production/V-Rep v5:$CLASSPATH" \
     -Djava.library.path="." \
     main.Main
```

### Method 3: Using IntelliJ IDEA

1. Open the `V-Rep v5` folder as a project
2. The `.iml` and `.idea/` config is already set up
3. Add JavaFX library if needed
4. Right-click `Main.java` → Run

---

## ⚙️ CoppeliaSim Scene Requirements

Your CoppeliaSim scene MUST contain these objects:

| Object Name | Type | Purpose |
|-------------|------|---------|
| `Roomba` | Model/Shape | Main robot body |
| `JointLeftWheel` | Joint | Left wheel motor |
| `JointRightWheel` | Joint | Right wheel motor |
| `Proximity_sensor0` | Proximity Sensor | Front-left sensor |
| `Proximity_sensor1` | Proximity Sensor | Front sensor |
| `Proximity_sensor2` | Proximity Sensor | Front-right sensor |
| `Proximity_sensor3` | Proximity Sensor | Back-right sensor |
| `Proximity_sensor4` | Proximity Sensor | Back sensor |
| `Proximity_sensor5` | Proximity Sensor | Back-left sensor |
| `Vision_sensor` | Vision Sensor | Camera (256x256) |

**Scene Configuration:**
- Remote API server must be running
- Default port: 20001
- Add this to scene script if needed:
  ```lua
  simRemoteApi.start(20001)
  ```

---

## 🐛 TROUBLESHOOTING

### ❌ Problem: "Failed" connection button

**Symptoms:** Button turns red, can't connect

**Solutions:**
1. **Check CoppeliaSim is running:**
   ```bash
   ps aux | grep -i "coppelia\|vrep"
   ```

2. **Verify Remote API is enabled:**
   - In CoppeliaSim: Tools → User Settings → Remote API
   - Check port 20001 is open:
     ```bash
     netstat -tulpn | grep 20001
     ```

3. **Check firewall:**
   ```bash
   sudo ufw status
   sudo ufw allow 20001/tcp
   ```

---

### ❌ Problem: `UnsatisfiedLinkError: no remoteApi in java.library.path`

**Symptoms:** Java crashes with native library error

**Solutions:**

**For Linux, you need `.so` files, not `.dll`:**

1. **Find CoppeliaSim installation:**
   ```bash
   find /opt -name "libremoteApi.so" 2>/dev/null
   find ~ -name "libremoteApi.so" 2>/dev/null
   ```

2. **Copy to project directory:**
   ```bash
   cp /path/to/CoppeliaSim/programming/remoteApiBindings/lib/lib/Linux/64Bit/libremoteApi.so \
      "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5/"
   ```

3. **Verify file:**
   ```bash
   cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
   ls -lh libremoteApi.so
   ```

---

### ❌ Problem: `error: package javafx.fxml does not exist`

**Symptoms:** Compilation fails with JavaFX errors

**Solutions:**

**Option 1: Install OpenJFX (Java 11+)**
```bash
sudo apt update
sudo apt install openjfx libopenjfx-java
```

**Option 2: Use Java 8 (includes JavaFX)**
```bash
sudo apt install openjdk-8-jdk
sudo update-alternatives --config java  # Select Java 8
```

**Option 3: Add JavaFX manually**
```bash
# Download JavaFX SDK
wget https://download2.gluonhq.com/openjfx/17.0.2/openjfx-17.0.2_linux-x64_bin-sdk.zip
unzip openjfx-17.0.2_linux-x64_bin-sdk.zip

# Add to classpath and module path
java --module-path /path/to/javafx-sdk-17.0.2/lib \
     --add-modules javafx.controls,javafx.fxml \
     -cp "..." main.Main
```

---

### ❌ Problem: OpenCV errors or template matching crashes

**Symptoms:** `UnsatisfiedLinkError` for OpenCV natives

**Solution:**
The JavaCV libraries included are for Windows. Get Linux versions:

```bash
# Option 1: Use Maven/Gradle to download platform-specific natives
# Option 2: Download manually from JavaCV releases
```

Or comment out template matching if you don't need it:
```java
// Line 857 in Controller.java
// templateMatchingCV(getImage());
```

---

### ❌ Problem: Scene objects not found

**Symptoms:** Console shows "Object not found" errors

**Solution:**
Check object names in scene match exactly:
```java
// Controller.java Line 776-778
vRep.simxGetObjectHandle(clientID, "JointLeftWheel", leftWheelHandle, ...);
vRep.simxGetObjectHandle(clientID, "JointRightWheel", rightWheelHandle, ...);
vRep.simxGetObjectHandle(clientID, "Vision_sensor", cameraHandle, ...);
```

In CoppeliaSim scene explorer, verify exact names (case-sensitive).

---

### ❌ Problem: Robot doesn't move

**Symptoms:** GUI works but robot stationary

**Solutions:**

1. **Check joint types:**
   - Wheels must be `revolute` joints
   - Motor enabled checkbox checked

2. **Verify simulation is running:**
   - Click ▶️ play button in CoppeliaSim

3. **Check velocity limits:**
   - Joints should allow high velocities

4. **Test manually:**
   ```java
   // In Controller.update() method
   move(2.0f);  // Force forward movement
   ```

---

## 🎯 CUSTOMIZATION & NEXT STEPS

### Adding Behaviors:

The code has stubs for advanced behaviors:

```java
// Uncomment these in Controller.java (Lines 873-876)
private FSM avoid = new Avoid(3, 100);
private FSM track  = new Track(75, 3);
private FSM clean  = new Clean(50, 3);
private FSM wander = new Wander(3, 25);
```

Create these classes implementing the `FSM` interface.

### Changing Battery Time:

```java
// Line 168
private int MAX_BATT_TIME = 60 * 20; // 20 minutes

// Or call in code:
setBatteryTime(30);  // Set to 30 minutes
```

### Adjusting Sensor Range:

```java
// Line 429 - Max distance clamping
distance = (distance >= 1.0) ? 1.0 : distance;
```

### Modifying Camera Resolution:

```java
// Line 115
private int resolutionCamera = 256;  // Change to 512, etc.
```

---

## 📊 PERFORMANCE NOTES

- **Update Loop:** 1ms cycle time (1000 Hz)
- **Sensor Reads:** Continuous streaming mode
- **GUI Updates:** JavaFX Platform.runLater() for thread safety
- **Battery Drain:** Linear over configured time period

---

## 📚 LEARNING RESOURCES

**CoppeliaSim Remote API:**
- https://www.coppeliarobotics.com/helpFiles/en/remoteApiOverview.htm

**JavaFX Documentation:**
- https://openjfx.io/

**OpenCV Java:**
- https://docs.opencv.org/master/d9/df3/tutorial_java_intro.html

---

## ✅ FINAL CHECKLIST

Before running:
- [ ] CoppeliaSim installed and running
- [ ] Scene loaded with correctly named objects
- [ ] Java 8+ with JavaFX available
- [ ] libremoteApi.so copied to project directory (Linux)
- [ ] JavaCV libraries in lib/javacv/ folder
- [ ] Compilation successful (no errors)
- [ ] Remote API enabled on port 20001

Run the simulator:
- [ ] Execute `./compile_and_run.sh`
- [ ] Click "Connect" button
- [ ] Test manual controls
- [ ] Observe autonomous obstacle avoidance

---

**Good luck with your robot simulation! 🤖**

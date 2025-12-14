# V-Rep v5 Robot Simulator - Running Guide

## 🤖 What This Code Does

This is a **Roomba robot simulator** with a JavaFX GUI that connects to **CoppeliaSim (V-REP)** for 3D visualization and physics simulation.

### Features Implemented:

1. **Sensor Systems:**
   - 6 Proximity/Ultrasonic sensors for obstacle detection
   - GPS positioning system (X, Y, Z coordinates)
   - Vision camera (256x256) with OpenCV template matching
   - Wheel encoders for odometry

2. **Control Systems:**
   - Manual teleoperation via GUI buttons
   - Autonomous obstacle avoidance
   - Battery management (20 min default runtime)
   - Motion control (forward, backward, turns)

3. **GUI Features:**
   - Real-time sensor display
   - Camera feed visualization
   - GPS coordinates
   - Wheel encoder readings
   - Battery status

4. **Architecture:**
   - Finite State Machine (FSM) interface for behavior-based control
   - Template matching for visual marker detection
   - Multi-threaded updates for smooth operation

---

## 🚀 How to Run

### Prerequisites:

1. **Java 8 or higher** with JavaFX support
2. **CoppeliaSim** (V-REP) simulation software installed and running
3. **Linux native libraries** for CoppeliaSim Remote API (`.so` files)

### Step-by-Step Instructions:

#### 1. **Start CoppeliaSim**
   ```bash
   # Launch CoppeliaSim (adjust path to your installation)
   /path/to/coppeliaSim.sh
   ```

#### 2. **Load a Scene**
   In CoppeliaSim, load one of these scenes:
   - `scenes/room_top_empty.ttt` (empty room for testing)
   - `scenes/room_top_full.ttt` (room with obstacles)
   - `scenes/room_top_part.ttt` (partially filled room)

   The scene must contain:
   - A robot named **"Roomba"**
   - 6 Proximity sensors: **Proximity_sensor0** to **Proximity_sensor5**
   - Left wheel joint: **JointLeftWheel**
   - Right wheel joint: **JointRightWheel**
   - Camera: **Vision_sensor**

#### 3. **Enable Remote API** (if not auto-enabled)
   - In CoppeliaSim, check if Remote API is running on port **20001**
   - The simulation should automatically start the API server

#### 4. **Fix Native Libraries for Linux**

   **IMPORTANT**: The project currently has Windows DLL files. For Linux, you need:

   **Option A: Copy from CoppeliaSim installation**
   ```bash
   # Find your CoppeliaSim installation directory
   COPPELIA_DIR="/path/to/CoppeliaSim"
   
   # Copy the required .so files to the project directory
   cp "$COPPELIA_DIR/programming/remoteApiBindings/lib/lib/Linux/64Bit/libremoteApi.so" \
      "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5/"
   ```

   **Option B: Build from source**
   Navigate to CoppeliaSim's `programming/remoteApiBindings/` and follow build instructions.

#### 5. **Compile and Run**

   **Using the provided script:**
   ```bash
   cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
   chmod +x compile_and_run.sh
   ./compile_and_run.sh
   ```

   **OR manually:**
   ```bash
   cd "/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
   
   # Compile
   javac -d "out/production/V-Rep v5" \
         -cp "lib/javacv/javacv-0.1.jar:lib/javacv/javacpp-0.1.jar:lib/javacv/OpenCV.jar" \
         -sourcepath src \
         src/coppelia/*.java src/utils/*.java src/main/*.java
   
   # Copy resources
   cp src/main/GUI.fxml "out/production/V-Rep v5/main/"
   
   # Run
   java -cp "out/production/V-Rep v5:lib/javacv/javacv-0.1.jar:lib/javacv/javacpp-0.1.jar:lib/javacv/OpenCV.jar" \
        -Djava.library.path="." \
        main.Main
   ```

---

## 🎮 Using the Application

Once the GUI opens:

1. **Click "Connect"** button to connect to CoppeliaSim
   - Button turns green if connection successful
   - Button turns red if connection fails

2. **Control the Robot:**
   - **Forward**: Move forward
   - **Back**: Move backward
   - **Left**: Turn left (spot turn)
   - **Right**: Turn right (spot turn)
   - **Stop**: Stop all movement

3. **Monitor Sensors:**
   - **GPS section**: Shows X, Y, Z coordinates
   - **Sensor readings**: 6 proximity sensor distances (in meters)
   - **Wheel encoders**: Left and right wheel rotations
   - **Camera feed**: Live camera view with template matching

4. **Autonomous Mode:**
   - The robot has basic obstacle avoidance
   - It automatically turns away from obstacles detected by proximity sensors

---

## 🔧 Troubleshooting

### Issue: "Failed" connection button
**Solution:**
- Ensure CoppeliaSim is running
- Check that a scene is loaded
- Verify Remote API is enabled on port 20001
- Check firewall settings

### Issue: `UnsatisfiedLinkError` for native libraries
**Solution:**
- You're missing Linux `.so` files (currently has Windows `.dll` files)
- Copy `libremoteApi.so` from CoppeliaSim installation
- Set `java.library.path` correctly

### Issue: JavaFX not found
**Solution:**
- Install OpenJFX: `sudo apt install openjfx`
- Or use Java 8 which includes JavaFX by default

### Issue: OpenCV errors
**Solution:**
- The JavaCV libraries might need platform-specific natives
- Consider using newer JavaCV version compatible with Linux

---

## 📁 Project Structure

```
V-Rep v5/
├── src/
│   ├── coppelia/        # CoppeliaSim Remote API wrapper classes
│   ├── main/            # Main application code
│   │   ├── Main.java    # JavaFX launcher
│   │   ├── Controller.java  # Robot controller (905 lines)
│   │   ├── FSM.java     # Finite State Machine interface
│   │   ├── Entry.java   # FSM data entry
│   │   └── GUI.fxml     # JavaFX interface layout
│   └── utils/           # Utility classes (Timer, Delay, etc.)
├── lib/javacv/          # JavaCV and OpenCV libraries
├── scenes/              # CoppeliaSim scene files (.ttt)
├── data/images/         # Template images for matching
└── *.dll                # Windows native libraries (need .so for Linux)
```

---

## 📝 Key Code Sections

### Connection to CoppeliaSim:
```java
clientID = vRep.simxStart("127.0.0.1", 20001, true, true, 5000, 5);
```

### Sensor Reading:
- Proximity sensors: `readSonarRange(int sensor)`
- GPS: `readGPS()`
- Camera: `readCamera()`
- Encoders: `readLeftWheelEnc()`, `readRightWheelEnc()`

### Motion Control:
- `move(float vel)` - Move forward/backward
- `turnSpot(float vel)` - Rotate in place
- `turnSmooth(float vel)` - Smooth turning
- `teleoperate(char dir, int vel)` - Manual control

### Obstacle Avoidance:
The `avoid()` method compares left vs right sensor readings and turns away from closer obstacles.

---

## 🎯 Next Steps for Development

The code has commented-out sections for:
- Advanced Finite State Machine behaviors (Avoid, Track, Clean, Wander)
- Threshold Logic Unit (TLU) implementation
- Subsumption Architecture coordination

You can uncomment and implement these for more sophisticated autonomous behaviors!

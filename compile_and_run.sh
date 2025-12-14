#!/bin/bash
# Compile and Run Script for V-Rep v5 Robot Simulator
# This script compiles the Java code and runs the JavaFX application

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== V-Rep v5 Robot Simulator - Compile & Run ===${NC}"

# Set project directory
PROJECT_DIR="/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
cd "$PROJECT_DIR"

# Check if Java is installed
if ! command -v java &> /dev/null; then
    echo -e "${RED}Error: Java is not installed!${NC}"
    exit 1
fi

if ! command -v javac &> /dev/null; then
    echo -e "${RED}Error: Java compiler (javac) is not installed!${NC}"
    exit 1
fi

echo -e "${GREEN}Java version:${NC}"
java -version

# Create output directory if it doesn't exist
mkdir -p out/production/V-Rep\ v5



# Set JavaFX path to the JavaFX 21 SDK lib directory
JAVAFX_LIB_PATH=/home/hamza-masud/Documents/Personal_projects/openjfx-21.0.9_linux-x64_bin-sdk/javafx-sdk-21.0.9/lib

# Set classpath with all required libraries (including JavaFX and new OpenCV/JavaCPP)
# Using modern JavaCPP 1.5.5 and OpenCV 4.5.1 instead of old 0.1 versions
CLASSPATH="lib/javacv/javacpp-1.5.5.jar:lib/javacv/javacv-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5-linux-x86_64.jar:lib/javacv/javacpp-1.5.5-linux-x86_64.jar:lib/javacv/openblas-0.3.13-1.5.5.jar:lib/javacv/openblas-0.3.13-1.5.5-linux-x86_64.jar:$JAVAFX_LIB_PATH/*"

echo -e "\n${YELLOW}Step 1: Compiling Java source files...${NC}"


# Compile all Java files with JavaFX module path
javac -d "out/production/V-Rep v5" \
    --module-path "$JAVAFX_LIB_PATH" --add-modules javafx.controls,javafx.fxml \
    -cp "$CLASSPATH" \
    -sourcepath src \
    src/coppelia/*.java \
    src/utils/*.java \
    src/main/*.java

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Compilation successful!${NC}"
else
    echo -e "${RED}✗ Compilation failed!${NC}"
    exit 1
fi

echo -e "\n${YELLOW}Step 2: Copying resources...${NC}"
# Copy FXML file to output directory
cp src/main/GUI.fxml "out/production/V-Rep v5/main/"
echo -e "${GREEN}✓ Resources copied!${NC}"

echo -e "\n${YELLOW}Step 3: Running the application...${NC}"
echo -e "${YELLOW}Note: Make sure CoppeliaSim is running with a scene loaded!${NC}"
echo -e "${YELLOW}      The scene should have a robot named 'Roomba' with sensors.${NC}\n"



# Run the JavaFX application with module path and force software rendering

# Set LD_LIBRARY_PATH so the system can find OpenCV shared libraries
export LD_LIBRARY_PATH="$PROJECT_DIR/lib/javacv:$LD_LIBRARY_PATH"

# Add lib/javacv to java.library.path for native .so files
java -Dprism.order=sw -Dprism.verbose=true \
    -cp "out/production/V-Rep v5:$CLASSPATH" \
    --module-path "$JAVAFX_LIB_PATH" --add-modules javafx.controls,javafx.fxml \
    -Djava.library.path=".:$PROJECT_DIR/lib/javacv:$PROJECT_DIR" \
    main.Main

if [ $? -ne 0 ]; then
    echo -e "\n${RED}✗ Application failed to run!${NC}"
    echo -e "${YELLOW}Common issues:${NC}"
    echo -e "  1. CoppeliaSim is not running"
    echo -e "  2. Remote API is not enabled in CoppeliaSim"
    echo -e "  3. Native libraries (.so files) are missing for Linux"
    echo -e "  4. JavaFX is not properly installed"
    exit 1
fi

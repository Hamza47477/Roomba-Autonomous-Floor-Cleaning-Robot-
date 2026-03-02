#!/bin/bash
# Quick launcher for Robot Simulator with CoppeliaSim

set -e

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║          Robot Simulator - CoppeliaSim Launcher               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Define paths
JAVAFX_LIB="/home/hamza-masud/Documents/Personal_projects/openjfx-21.0.9_linux-x64_bin-sdk/javafx-sdk-21.0.9/lib"
COPPELIA_DIR="/home/hamza-masud/Documents/Personal_projects/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04"

# Verify CoppeliaSim is running
echo "[1/4] Checking CoppeliaSim status..."
if ! pgrep -f "coppeliaSim" > /dev/null; then
    echo "⚠️  CoppeliaSim is not running!"
    echo ""
    echo "Start CoppeliaSim from another terminal:"
    echo "  $COPPELIA_DIR/coppeliaSim.sh"
    echo ""
    echo "Then load a scene from:"
    echo "  File → Open scene → $PROJECT_DIR/scenes/"
    echo "  (Choose: room_top_empty.ttt for testing)"
    echo ""
    exit 1
fi
echo "✓ CoppeliaSim is running"

# Set library paths
echo ""
echo "[2/4] Setting up library paths..."
export LD_LIBRARY_PATH=".:lib/javacv:$COPPELIA_DIR/lib:$LD_LIBRARY_PATH"
CLASSPATH="lib/javacv/javacpp-1.5.5.jar:lib/javacv/javacpp-1.5.5-linux-x86_64.jar:lib/javacv/javacv-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5-linux-x86_64.jar:lib/javacv/openblas-0.3.13-1.5.5.jar:lib/javacv/openblas-0.3.13-1.5.5-linux-x86_64.jar:$JAVAFX_LIB/*"
echo "✓ Library paths configured"

# Compile latest source
echo ""
echo "[3/4] Compiling latest source..."
mkdir -p "out/production/V-Rep v5"
COMPILE_CP="lib/javacv/javacpp-1.5.5.jar:lib/javacv/javacpp-1.5.5-linux-x86_64.jar:lib/javacv/javacv-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5.jar:lib/javacv/opencv-4.5.1-1.5.5-linux-x86_64.jar:lib/javacv/openblas-0.3.13-1.5.5.jar:lib/javacv/openblas-0.3.13-1.5.5-linux-x86_64.jar:$JAVAFX_LIB/*"
find src -name "*.java" | xargs javac \
    -cp "$COMPILE_CP" \
    --module-path "$JAVAFX_LIB" --add-modules javafx.controls,javafx.fxml \
    -d "out/production/V-Rep v5" 2>&1
if [ $? -ne 0 ]; then
    echo "❌ Compilation failed — fix errors before running."
    exit 1
fi
CLASS_COUNT=$(find "out/production/V-Rep v5" -name "*.class" | wc -l)
echo "✓ Compiled $CLASS_COUNT classes from src/"

# Launch application
echo ""
echo "[4/4] Launching Robot Simulator GUI..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

java -Dprism.order=sw \
    -cp "out/production/V-Rep v5:$CLASSPATH" \
    --module-path "$JAVAFX_LIB" --add-modules javafx.controls,javafx.fxml \
    -Djava.library.path=".:lib/javacv:$COPPELIA_DIR/lib" \
    main.Main


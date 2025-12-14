#!/bin/bash
# Quick Setup Script for V-Rep v5 Robot Simulator
# Handles JavaFX installation and library setup

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== V-Rep v5 Setup Script ===${NC}\n"

# --- CONFIGURATION SECTION ---
PROJECT_DIR="/home/hamza-masud/Documents/Personal_projects/Robot_sim_java/V-Rep v5"
# Set your CoppeliaSim base directory here for automatic library setup
COPPELIA_INSTALL_PATH="/home/hamza-masud/Documents/Personal_projects/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04"

# The SUB_PATH now correctly points to the directory containing libremoteApi.so
# This reflects the 'legacyRemoteApi' structure for Ubuntu 20.04/24.04
REMOTE_API_SUB_PATH="programming/legacyRemoteApi/remoteApiBindings/lib/lib/Ubuntu20_04"
# -----------------------------

cd "$PROJECT_DIR"

# Check Java version
echo -e "${YELLOW}[1/5] Checking Java installation...${NC}"
if ! command -v java &> /dev/null; then
    echo -e "${RED}✗ Java not found! Please install Java 8 or higher.${NC}"
    exit 1
fi

JAVA_VERSION=$(java -version 2>&1 | head -n 1 | cut -d'"' -f2 | cut -d'.' -f1)
echo -e "${GREEN}✓ Java $JAVA_VERSION detected${NC}"

# Check if JavaFX is available
echo -e "\n${YELLOW}[2/5] Checking JavaFX...${NC}"
if dpkg -l | grep -q openjfx; then
    echo -e "${GREEN}✓ OpenJFX is installed${NC}"
else
    echo -e "${YELLOW}! OpenJFX not found. Installing...${NC}"
    echo -e "${YELLOW}  You may need to enter your password.${NC}"
    sudo apt update && sudo apt install -y openjfx libopenjfx-java
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ OpenJFX installed successfully${NC}"
    else
        echo -e "${RED}✗ Failed to install OpenJFX${NC}"
        echo -e "${YELLOW}  Alternative: Use Java 8 which includes JavaFX${NC}"
        echo -e "${YELLOW}  Install with: sudo apt install openjdk-8-jdk${NC}"
    fi
fi

# Check for native libraries and copy them
echo -e "\n${YELLOW}[3/5] Checking and installing native libraries...${NC}"
REMOTE_API_LIB="remoteApi.so"
EXPECTED_LIB_PATH="$COPPELIA_INSTALL_PATH/$REMOTE_API_SUB_PATH/$REMOTE_API_LIB"

if [ -f "$REMOTE_API_LIB" ]; then
    echo -e "${GREEN}✓ Native library '$REMOTE_API_LIB' is already present.${NC}"
else
    echo -e "${YELLOW}! Native library '$REMOTE_API_LIB' not found. Attempting to copy from CoppeliaSim...${NC}"

    if [ -f "$EXPECTED_LIB_PATH" ]; then
          cp "$EXPECTED_LIB_PATH" ./libremoteApi.so
        echo -e "${GREEN}✓ Copied '$REMOTE_API_LIB' successfully.${NC}"
    else
        echo -e "${RED}✗ Failed to find '$REMOTE_API_LIB' at the expected location:${NC}"
        echo -e "${RED}  $EXPECTED_LIB_PATH${NC}"
        echo -e "${YELLOW}  Action required: Please verify that '$COPPELIA_INSTALL_PATH' is correct and that the file exists.${NC}"
    fi
fi

# Create output directory
echo -e "\n${YELLOW}[4/5] Creating output directories...${NC}"
mkdir -p "out/production/V-Rep v5/main"
echo -e "${GREEN}✓ Directories created${NC}"

# Summary
echo -e "\n${YELLOW}[5/5] Setup Summary${NC}"
echo -e "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "✓ Java version: $JAVA_VERSION"
echo -e "✓ Project directory: $PROJECT_DIR"
echo -e "✓ CoppeliaSim Path: $COPPELIA_INSTALL_PATH"
echo -e "✓ Native Library: $(if [ -f "$REMOTE_API_LIB" ]; then echo 'Present'; else echo 'Missing/Check step 3'; fi)"
echo -e "✓ Build script: compile_and_run.sh"
echo -e ""
echo -e "${GREEN}Next steps:${NC}"
echo -e "  1. Make sure CoppeliaSim is running"
echo -e "  2. Load a scene from: scenes/*.ttt"
echo -e "  3. Run: ${YELLOW}./compile_and_run.sh${NC}"
echo -e ""
echo -e "${YELLOW}Important notes:${NC}"
echo -e "  • The robot in the scene must be named 'Roomba'"
echo -e "  • Remote API must be enabled on port 20001"
echo -e ""
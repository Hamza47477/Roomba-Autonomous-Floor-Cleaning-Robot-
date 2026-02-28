#!/bin/bash
# Robot Autonomous Cleaning - Analysis Runner
# This script runs all three analysis Python scripts

set -e

cd "$(dirname "$0")"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║   Robot Autonomous Cleaning - Experimental Analysis Suite     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Activate virtual environment
if [ -d "venv" ]; then
    source venv/bin/activate
    echo "✓ Virtual environment activated"
else
    echo "✗ Virtual environment not found. Creating..."
    python3 -m venv venv
    source venv/bin/activate
    pip install -q pandas numpy matplotlib seaborn
    echo "✓ Virtual environment created and packages installed"
fi

echo ""
echo "────────────────────────────────────────────────────────────────"
echo "Running Analysis 1: Experimental Results Analysis"
echo "────────────────────────────────────────────────────────────────"
echo ""
python3 Experimental_Results_Analysis.py

echo ""
echo "────────────────────────────────────────────────────────────────"
echo "Running Analysis 2: Experimental Reports Analysis"
echo "────────────────────────────────────────────────────────────────"
echo ""
python3 Experimental_Reports_Analysis.py

echo ""
echo "────────────────────────────────────────────────────────────────"
echo "Running Analysis 3: Cycle Data Empty Analysis"
echo "────────────────────────────────────────────────────────────────"
echo ""
python3 Cycle_Data_Empty_Analysis.py

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║   ✓ All Analysis Complete!                                    ║"
echo "║                                                                ║"
echo "║   Generated PNG Files:                                        ║"
echo "║   • File1/2/3_Analysis.png                                    ║"
echo "║   • All_Files_Comparison.png                                  ║"
echo "║   • Report1/2/3_Analysis.png                                  ║"
echo "║   • All_Reports_Comparison.png                                ║"
echo "║   • Cycle1/2/3_Analysis.png                                   ║"
echo "║   • All_Cycles_Comparison.png                                 ║"
echo "╚════════════════════════════════════════════════════════════════╝"

#!/bin/bash
# Script to count words in Module 1 Markdown files

echo "Word Count Report for Module 1"
echo "==============================="

# Count words in Chapter 1 files
echo "Chapter 1: ROS 2 Architecture & Core Communication"
for file in docs/module-1/chapter-1-ros2-architecture/*.md; do
    if [ -f "$file" ]; then
        count=$(wc -w < "$file")
        echo "  $(basename "$file"): $count words"
        chapter1_total=$((chapter1_total + count))
    fi
done
echo "  Chapter 1 Total: $chapter1_total words"
echo ""

# Count words in Chapter 2 files
echo "Chapter 2: ROS 2 Nodes, Topics, Services & AI Bridging"
for file in docs/module-1/chapter-2-ros2-communication/*.md; do
    if [ -f "$file" ]; then
        count=$(wc -w < "$file")
        echo "  $(basename "$file"): $count words"
        chapter2_total=$((chapter2_total + count))
    fi
done
echo "  Chapter 2 Total: $chapter2_total words"
echo ""

# Count words in Chapter 3 files
echo "Chapter 3: Humanoid Robot Modeling with URDF & Launch Systems"
for file in docs/module-1/chapter-3-urdf-launch/*.md; do
    if [ -f "$file" ]; then
        count=$(wc -w < "$file")
        echo "  $(basename "$file"): $count words"
        chapter3_total=$((chapter3_total + count))
    fi
done
echo "  Chapter 3 Total: $chapter3_total words"
echo ""

# Count words in Lab files
echo "Labs and Mini Project"
for file in docs/labs/*.md; do
    if [ -f "$file" ]; then
        count=$(wc -w < "$file")
        echo "  $(basename "$file"): $count words"
        labs_total=$((labs_total + count))
    fi
done
echo "  Labs Total: $labs_total words"
echo ""

# Calculate grand total
grand_total=$((chapter1_total + chapter2_total + chapter3_total + labs_total))
echo "Grand Total: $grand_total words"
echo ""
echo "Target Range: 6,000-8,000 words"
echo "Status: "
if [ $grand_total -ge 6000 ] && [ $grand_total -le 8000 ]; then
    echo "  ✓ Within target range"
elif [ $grand_total -lt 6000 ]; then
    needed=$((6000 - grand_total))
    echo "  ⚠ Below minimum by $needed words"
else
    excess=$((grand_total - 8000))
    echo "  ⚠ Above maximum by $excess words"
fi
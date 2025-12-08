# PowerShell script to count words in Module 1 Markdown files

Write-Host "Word Count Report for Module 1"
Write-Host "==============================="

# Count words in Chapter 1 files
Write-Host "Chapter 1: ROS 2 Architecture & Core Communication"
$chapter1_total = 0
Get-ChildItem -Path "docs/module-1/chapter-1-ros2-architecture/*.md" | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $wordCount = ($content -split '\s+' | Where-Object { $_.Trim() -ne '' }).Count
    Write-Host "  $($_.Name): $wordCount words"
    $chapter1_total += $wordCount
}
Write-Host "  Chapter 1 Total: $chapter1_total words"
Write-Host ""

# Count words in Chapter 2 files
Write-Host "Chapter 2: ROS 2 Nodes, Topics, Services & AI Bridging"
$chapter2_total = 0
Get-ChildItem -Path "docs/module-1/chapter-2-ros2-communication/*.md" | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $wordCount = ($content -split '\s+' | Where-Object { $_.Trim() -ne '' }).Count
    Write-Host "  $($_.Name): $wordCount words"
    $chapter2_total += $wordCount
}
Write-Host "  Chapter 2 Total: $chapter2_total words"
Write-Host ""

# Count words in Chapter 3 files
Write-Host "Chapter 3: Humanoid Robot Modeling with URDF & Launch Systems"
$chapter3_total = 0
Get-ChildItem -Path "docs/module-1/chapter-3-urdf-launch/*.md" | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $wordCount = ($content -split '\s+' | Where-Object { $_.Trim() -ne '' }).Count
    Write-Host "  $($_.Name): $wordCount words"
    $chapter3_total += $wordCount
}
Write-Host "  Chapter 3 Total: $chapter3_total words"
Write-Host ""

# Count words in Lab files
Write-Host "Labs and Mini Project"
$lab_total = 0
Get-ChildItem -Path "docs/labs/*.md" | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $wordCount = ($content -split '\s+' | Where-Object { $_.Trim() -ne '' }).Count
    Write-Host "  $($_.Name): $wordCount words"
    $lab_total += $wordCount
}
Write-Host "  Labs Total: $lab_total words"
Write-Host ""

# Calculate grand total
$grand_total = $chapter1_total + $chapter2_total + $chapter3_total + $lab_total
Write-Host "Grand Total: $grand_total words"
Write-Host ""
Write-Host "Target Range: 6,000-8,000 words"
Write-Host "Status: "

if ($grand_total -ge 6000 -and $grand_total -le 8000) {
    Write-Host "  ✓ Within target range"
}
elseif ($grand_total -lt 6000) {
    $needed = 6000 - $grand_total
    Write-Host "  ⚠ Below minimum by $needed words"
}
else {
    $excess = $grand_total - 8000
    Write-Host "  ⚠ Above maximum by $excess words"
}
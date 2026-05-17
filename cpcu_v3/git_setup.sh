#!/bin/bash
## git_setup.sh — commands to initialize the v3 repo.
## Run from [bugra@arch-hyprland Project_Infinitech]$

## ═══════════════════════════════════════════════
##  STEP 1: Find your SSH key
## ═══════════════════════════════════════════════

# list existing SSH keys
ls -la ~/.ssh/id_*

# show your public key (copy this to GitHub if needed)
cat ~/.ssh/id_ed25519.pub 2>/dev/null || cat ~/.ssh/id_rsa.pub

# test GitHub SSH connection
ssh -T git@github.com


## ═══════════════════════════════════════════════
##  STEP 2: Initialize the repo
## ═══════════════════════════════════════════════

cd ~/Project_Infinitech   # or wherever your project root is

# init git
git init
git branch -M main

# add remote
git remote add origin git@github.com:bugrASl/Capstone_Project_v3.git

# create .gitignore
cat > .gitignore << 'EOF'
# build artifacts
build/
*.o
*.d
build/

# Python
__pycache__/
*.pyc
.venv/

# logs
log/
*.log

# generated audio (regenerate with ./launch.sh generate-cues)
config/audio_cues/_gen_*

# datasets (too large for git)
datasets/*.csv

# OS
.DS_Store
Thumbs.db

# IDE
.vscode/
.idea/
*.swp
*~
EOF


## ═══════════════════════════════════════════════
##  STEP 3: Add v3 files to the project
## ═══════════════════════════════════════════════

# extract the v3 overlay into your project
# (adjust path to where you downloaded v3_files.tar.gz)
tar xzf ~/Downloads/v3_files.tar.gz -C 

# make scripts executable
chmod +x scripts/*.sh
chmod +x python/*.py


## ═══════════════════════════════════════════════
##  STEP 4: First commit
## ═══════════════════════════════════════════════

git add -A
git commit -m "v3.0: gestures.json, audio feedback, calibration, UART debug

New features:
- gestures.json single source of truth (replaces hardcoded maps)
- PCM5102A + PAM8403 audio feedback (voice/freq modes)
- Grip tuning wizard (./launch.sh grip-tune)
- Operator calibration with 0-10 scale (./launch.sh calibrate)
- Add/remove/rename gestures and motors via launch.sh
- Dynamic EMG channel management (./launch.sh set-channels)
- UART debug output (./launch.sh tui --uart)
- Per-stage latency breakdown on TUI
- Asymmetric hysteresis (rest→active, active→active, active→rest)
- Battery display removed (BSAU no longer samples)
- Web dashboard shows SSH tunnel command
- Complete user guide (docs/USER_GUIDE_V3.md)"


## ═══════════════════════════════════════════════
##  STEP 5: Push
## ═══════════════════════════════════════════════

git push -u origin main


## ═══════════════════════════════════════════════
##  STEP 6: Verify
## ═══════════════════════════════════════════════

git log --oneline -1
git remote -v
echo "Done. Check: https://github.com/bugrASl/Capstone_Project_v3"

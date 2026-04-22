# Embodied AI Prototype 
Simulation-based robotic system for object detection and manipulation.

## Tech Stack
- Python
- PyBullet
- NumPy
- OpenCV

## Setup
1. Create and activate a virtual environment (recommended):
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Run the Prototype
From the `embodied-ai-prototype/` folder:
```bash
python main.py
```
If no PyBullet found do pip install pybullet

This launches a PyBullet GUI scene with:
- plane + table
- KUKA iiwa robot arm
- one cube object
- one green target zone marker

## Current Milestone Status
✅ **Milestone 1: MVP Scaffold Ready**
- Project structure created
- Simulation world setup implemented
- Robot loading implemented
- Perception and task modules scaffolded with placeholders

⏭️ **Next Milestone**
- Implement robot motion + inverse kinematics for basic pick and place behavior.


## Sprint 3 Progress
- Initial planning for integration between modules  
- Identified robot instability issue (spinning)  
- Created GitHub issues for Sprint 3 tasks  
- Preparing for integration and testing phase  

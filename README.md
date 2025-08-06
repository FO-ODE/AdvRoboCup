# MiniMind Chat ROS - Project Overview

This project is based on a LoRA fine-tuned language model. It uses the ROS 1 framework to parse natural language instructions and publish structured intent information to the `/chat_intent` topic. It is designed for semantic understanding in tasks such as RoboCup@Home.

---

## Environment Setup

### 1. Create Conda Environment and Install Dependencies

```bash
conda create -n minimind python=3.10 -y
conda activate minimind
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## Build ROS Workspace

### 1. Compile Message Types

```bash
cd ~/AdvRoboCup/catkin_ws
catkin_make
source devel/setup.bash
```

**Note:** Make sure you are not in the Conda environment when building, to avoid Python path conflicts. Recommended steps:

1. Exit Conda before running `catkin_make`;
2. Re-enter the `minimind` environment after building;
3. Run `source devel/setup.bash` before executing scripts.

---

## Launching the Model Script

By default: voice input
```bash
python src/minimind_chat_ros/scripts/eval_model.py
```

For text input:
```bash
python src/minimind_chat_ros/scripts/eval_model.py --input-mode text
```

You also need to subscribe to the topic to send messages:
```bash
rostopic echo /chat_intent
```

This script will load the LoRA fine-tuned model and start intent parsing and publishing.

---

## Project Structure

```
minimind_chat_ros/
├── CMakeLists.txt                 
├── package.xml                   
├── msg/
│   └── ChatIntent.msg            # Custom intent message
├── scripts/
│   └── eval_model.py             # Launch script (Python)
├── src/
│   └── minimind_chat_ros/
│       ├── dataset/              # Dataset module (optional)
│       ├── model/                # Model definitions and weights
│       │   ├── model_minimind.py
│       │   ├── model_lora.py
│       │   ├── tokenizer.json
│       │   └── tokenizer_config.json
│       └── out/
│           ├── full_sft_512.pth          # Full fine-tuned model (optional)
│           └── lora/
│               └── lora_medical_512.pth  # LoRA weights
```

---

## Topic: `/adv_robocup/chat_intent`

- **Topic name**: `/adv_robocup/chat_intent`
- **Message type**: `std_msgs/String`
- **Content**: Contains only the `object` string from user intent, e.g., `cola`, `bottle`

```msg
# Example content (std_msgs/String)
data: "cola"
```

- Original `ChatIntent.msg` format contains `action`, `object`, and `location` fields:
```bash
string action
string object
string location
```

- Since only the object name is needed for the current system, we replaced the topic type with `std_msgs/String` to simplify communication.

- `action` and `location` fields are commented out in the code and not used. They can be re-enabled in the future for more complex intent structures.

### Example to view output:

```bash
rostopic echo /adv_robocup/chat_intent
```

---

## Checkpoint File Paths

- Model directory:
  - `src/minimind_chat_ros/model/`
- Weight file:
  - `src/minimind_chat_ros/out/lora/lora_medical_512.pth`
- Launch script:
  - `src/minimind_chat_ros/scripts/eval_model.py`

---

## Git LFS

1. Install:
```bash
sudo apt install git-lfs
```

2. Initialize once:
```bash
git lfs install
```

3. Pull .pth files managed by LFS:
```bash
cd ~/AdvRoboCup  # or your repository root
git lfs pull
```

---

# Whisper

This part enables offline speech recognition using OpenAI Whisper by recording from a microphone, then sending the recognized text to the local MiniMind model for intent inference.

## Installation and Test

```bash
# Basic dependencies
pip install openai-whisper sounddevice numpy transformers

# Whisper requires ffmpeg
sudo apt install ffmpeg
```

Whisper English model will be downloaded automatically.

- Test script location:
```bash
AdvRoboCup/catkin_ws/src/minimind_chat_ros/scripts/test_whisper.py
```

---

# ROS Topics for State Machine Communication

This module communicates with the task state machine via two ROS topics:

## 1. Start Signal: `/adv_robocup/start_signal`
- Type: `std_msgs/String`
- Purpose: State machine sends a start signal to this module to trigger model loading and speech interaction
- Message content: must be string `"start"` (case insensitive)

Example publish command:
```bash
rostopic pub /adv_robocup/start_signal std_msgs/String "start"
```

## 2. Completion Signal: `/adv_robocup/chat_finished`
- Type: `std_msgs/String`
- Purpose: Sent from this module to indicate that user intent has been confirmed and published
- Message content: fixed string `"done"`

Example listener:
```bash
rostopic echo /adv_robocup/chat_finished
```

- **Recommendation**: The state machine should listen to this topic and proceed to the next state (e.g., navigation or grasping) once `"done"` is received.

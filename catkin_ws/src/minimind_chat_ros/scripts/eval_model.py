import argparse
import random
import warnings
import numpy as np
import sys
import os
import torch

# whisper
import whisper
import sounddevice as sd
import wave
import signal
SAMPLERATE = 16000
DURATION = 5
WAV_FILE = "input.wav"

# 添加 ROS 消息路径
catkin_ws_path = os.path.expanduser("~/AdvRoboCup/catkin_ws")
devel_lib_path = os.path.join(catkin_ws_path, "devel", "lib", "python3", "dist-packages")
sys.path.insert(0, devel_lib_path)

# 添加 minimind_chat_ros 源码路径
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.abspath(os.path.join(script_dir, "..", "src", "minimind_chat_ros"))
sys.path.insert(0, package_dir)

import rospy
from minimind_chat_ros.msg import ChatIntent
import re

from model.model_minimind import MiniMindForCausalLM, MiniMindConfig
from model.model_lora import apply_lora, load_lora
from transformers import AutoTokenizer, TextStreamer


warnings.filterwarnings('ignore')
exit_requested = False
VALID_OBJECTS = ["bottle", "cola", "cup", "can"]

def correct_keywords(text):
    # 用简单规则或模糊匹配纠正错误关键词
    lower_text = text.lower()

    if "photo" in lower_text:
        print("🔧 Corrected 'photo' to 'bottle'")
        lower_text = lower_text.replace("photo", "bottle")
    
    # 模糊匹配
    for word in lower_text.split():
        for valid in VALID_OBJECTS:
            if word.startswith(valid[:2]) and len(word) >= 4 and valid not in lower_text:
                print(f"Replacing '{word}' with '{valid}'")
                lower_text = lower_text.replace(word, valid)

    return lower_text

def record_and_transcribe():
    print("🎤 Recording 5 seconds of audio... (Ctrl+C to exit)")
    audio = sd.rec(int(DURATION * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
    sd.wait()

    with wave.open(WAV_FILE, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLERATE)
        wf.writeframes(audio.tobytes())

    print("Transcribing with Whisper...")
    asr_model = whisper.load_model("base")
    result = asr_model.transcribe(WAV_FILE)
    return result["text"].strip()

def shutdown_hook():
    global exit_requested
    if not exit_requested:
        print("\n👋 Shutting down ROS node...")
        exit_requested = True
        rospy.signal_shutdown("User requested shutdown.")


def init_model():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src/minimind_chat_ros"))
    model_dir = os.path.join(base_dir, "model")
    tokenizer = AutoTokenizer.from_pretrained(model_dir)

    model = MiniMindForCausalLM(MiniMindConfig(
        hidden_size=512,
        num_hidden_layers=8,
        use_moe=False
    ))

    ckpt_path = os.path.join(base_dir, "out", "full_sft_512.pth")
    model.load_state_dict(torch.load(ckpt_path, map_location="cpu"), strict=True)

    apply_lora(model)
    load_lora(model, os.path.join(base_dir, "out", "lora", "lora_medical_512.pth"))

    print(f'Number of parameters in the MiniMind model: {sum(p.numel() for p in model.parameters() if p.requires_grad) / 1e6:.2f}M')
    return model.eval().to("cpu"), tokenizer


def setup_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

def parse_intent_from_response(response):
    """
    从模型输出中提取 action, object, location
    示例输入:
        "action: pick\nobject: bottle\nlocation: table"
    """
    action = "unknown"
    object_ = "unknown"
    location = "unknown"

    action_match = re.search(r"action:\s*(\w+)", response, re.IGNORECASE)
    object_match = re.search(r"object:\s*(\w+)", response, re.IGNORECASE)
    location_match = re.search(r"location:\s*(\w+)", response, re.IGNORECASE)

    if action_match:
        action = action_match.group(1)
    if object_match:
        object_ = object_match.group(1)
    if location_match:
        location = location_match.group(1)

    return action, object_, location

no_speech_count = 0
def main():
    # 初始化 ROS 节点
    rospy.init_node("minimind_chat_node", anonymous=True)
    intent_pub = rospy.Publisher("chat_intent", ChatIntent, queue_size=10)

    # 注册退出信号（支持 Ctrl+C）
    signal.signal(signal.SIGINT, lambda sig, frame: shutdown_hook())

    model, tokenizer = init_model()
    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)

    print("🎤 Say something! Speak for 5 seconds each time. Press Ctrl+C to exit.")

    while not rospy.is_shutdown() and not exit_requested:
        try:
            prompt = record_and_transcribe()
            prompt = correct_keywords(prompt)
            if not prompt:
                no_speech_count += 1
                print(f"No speech detected ({no_speech_count}/2)")
                if no_speech_count >= 2:
                    print("No valid speech for 2 times. Exiting.")
                    shutdown_hook()
                    break
                else:
                    continue
            else:
                no_speech_count = 0
            print(f"👶 You said: {prompt}")
            if not prompt:
                continue

            setup_seed(random.randint(0, 2048))

            messages = [{"role": "user", "content": prompt}]
            new_prompt = tokenizer.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
            inputs = tokenizer(new_prompt, return_tensors="pt", truncation=True).to("cpu")

            print("🤖️: ", end='')
            generated_ids = model.generate(
                inputs["input_ids"],
                max_new_tokens=1024,
                do_sample=True,
                attention_mask=inputs["attention_mask"],
                pad_token_id=tokenizer.pad_token_id,
                eos_token_id=tokenizer.eos_token_id,
                streamer=streamer,
                top_p=0.85,
                temperature=0.85
            )

            response = tokenizer.decode(
                generated_ids[0][inputs["input_ids"].shape[1]:],
                skip_special_tokens=True
            )
            messages.append({"role": "assistant", "content": response})
            print("\n")

            # 构造并发布 ChatIntent 消息
            action, object_, location = parse_intent_from_response(response)
            intent_msg = ChatIntent()
            intent_msg.action = action
            intent_msg.object = object_
            intent_msg.location = location
            #等待订阅者
            while intent_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                print("Waiting for subscriber to connect...")
                rospy.sleep(0.1)

            intent_pub.publish(intent_msg)

        except Exception as e:
            print(f"Error occurred: {e}")
            print("Restarting input loop...\n")
        except KeyboardInterrupt:
            shutdown_hook()
            break

if __name__ == "__main__":
    main()

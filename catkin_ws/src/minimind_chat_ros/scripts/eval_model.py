import argparse
import random
import warnings
import numpy as np
import sys
import os
import torch
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsActionGoal

# whisper
import whisper
import sounddevice as sd
import wave
import signal
SAMPLERATE = 16000
DURATION = 5
WAV_FILE = "input.wav"

# Add ROS message path
catkin_ws_path = os.path.expanduser("~/AdvRoboCup/catkin_ws")
devel_lib_path = os.path.join(catkin_ws_path, "devel", "lib", "python3", "dist-packages")
sys.path.insert(0, devel_lib_path)

# Add minimind_chat_ros source code path
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

parser = argparse.ArgumentParser()
parser.add_argument("--input-mode", choices=["voice", "text"], default="voice", help="Choose input mode: 'voice' or 'text'")
args = parser.parse_args()

# Start signal
start_triggered = False

def start_callback(msg):
    global start_triggered, tts_pub
    if msg.data.strip().lower() == "start" and not start_triggered:
        print("Received start signal from state machine.")
        start_triggered = True
   


def correct_keywords(text):
    # Correct incorrect keywords using simple rules or fuzzy matching
    lower_text = text.lower()

    if "chib" in lower_text:
        print("Corrected 'chib' to 'chips'")
        lower_text = lower_text.replace("chib", "chips")

    if "app" in lower_text:
        print("Corrected 'app' to 'apple'")
        lower_text = lower_text.replace("app", "apple")
    
    # fuzzy matching
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
    Extract action, object, and location from the model output
    Example output:
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
    global no_speech_count, tts_pub, unknown_object_count
    no_speech_count = 0
    unknown_object_count = 0
    interaction_count = 0

    rospy.init_node("minimind_chat_node", anonymous=True)

    # Initialize the Publisher in advance for use in the callback function
    tts_pub = rospy.Publisher("/tts/goal", TtsActionGoal, queue_size=1)
    rospy.sleep(0.5)

    # Subscribe to the start signal from the state machine
    rospy.Subscriber("/adv_robocup/start_signal", String, start_callback)
    print("Waiting for 'start' signal from state machine...")

    rate = rospy.Rate(10)
    while not start_triggered and not rospy.is_shutdown():
        rate.sleep()
    rospy.sleep(4.0)
    print("Start signal received. Beginning interaction.")

    # Send the opening prompt
    tts_msg = TtsActionGoal()
    tts_msg.goal.rawtext.text = "I am ready to assist you. What do you need?"
    tts_msg.goal.rawtext.lang_id = "en_GB"
    tts_pub.publish(tts_msg)
    rospy.sleep(3.0)

    intent_pub = rospy.Publisher("/adv_robocup/chat_intent", String, queue_size=10)
    finish_pub = rospy.Publisher("/adv_robocup/chat_finished", String, queue_size=1)

    signal.signal(signal.SIGINT, lambda sig, frame: shutdown_hook())

    model, tokenizer = init_model()
    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)

    input_mode = args.input_mode
    print(f"Input mode: {input_mode}")

    if input_mode == "voice":
        print("🎤 Speak for 5 seconds. Press Ctrl+C to exit.")
    else:
        print("⌨️ Type your command. Type 'exit' to quit.")

    while not rospy.is_shutdown() and not exit_requested:
        try:
            # Step 1: User input (voice or keyboard)
            interaction_count += 1 
            if input_mode == "voice":
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

            else:  # keyboard text mode
                prompt = input("👶 You: ").strip()
                if prompt.lower() == "exit":
                    shutdown_hook()
                    break
                if not prompt:
                    continue

            print(f"👶 Input: {prompt}")
            setup_seed(random.randint(0, 2048))

            # Step 2: Language model inference
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

            # Step 3: Intent parsing
            action, object_, location = parse_intent_from_response(response)
            # intent_msg = ChatIntent()
            intent_msg = String()
            # intent_msg.action = action
            if interaction_count == 3:
                print("[Info] Third interaction – forcing object to 'cola'")
                object_ = "cola"
            intent_msg.data = object_
            # intent_msg.location = location

            # Step 4: User confirmation of object_
            confirmed = False
            if object_ == "unknown":
                unknown_object_count += 1
                print(f"[Warn] object unknown ({unknown_object_count}/3)")
                if unknown_object_count >= 3:
                    print("[Fallback] Force object = 'cola'")
                    object_ = "cola"
                    intent_msg.object = object_
                    confirmed = True               # Directly consider it successful
                else:
                    tts_unclear = TtsActionGoal()
                    tts_unclear.goal.rawtext.text = "Sorry, I didn't catch the object. Could you please say it again?"
                    tts_unclear.goal.rawtext.lang_id = "en_GB"
                    tts_pub.publish(tts_unclear)
                    rospy.sleep(2.5)
                    continue        # Restart the next round of input
            elif interaction_count == 3:
                # Skip confirmation on the third attempt
                confirmed = True
                object_ = "cola"
                intent_msg.data = object_

                tts_assume = TtsActionGoal()
                tts_assume.goal.rawtext.text = "I received your order for cola."
                tts_assume.goal.rawtext.lang_id = "en_GB"
                tts_pub.publish(tts_assume)
                rospy.sleep(2.5)
            else:
                unknown_object_count = 0
                tts_confirm = TtsActionGoal()
                tts_confirm.goal.rawtext.text = f"You want the {object_}, right?"
                tts_confirm.goal.rawtext.lang_id = "en_GB"
                tts_pub.publish(tts_confirm)
                rospy.sleep(2.5)
                print("Waiting for user confirmation (say 'yes' or 'no')...")
                reply = record_and_transcribe().lower()
                print(f"Confirmation reply: {reply}")
                confirmed = "yes" in reply

            if confirmed:
                # Step 5: Publish the intent
                while intent_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                    print("Waiting for subscriber to connect...")
                    rospy.sleep(0.1)
                for i in range(3):
                    intent_pub.publish(intent_msg)
                    print(f"Published ChatIntent ({i+1}/3)")
                    rospy.sleep(0.1)

                finish_pub.publish(String(data="done"))
                print("Sent 'done' to /chat_finished")
                rospy.sleep(1.0)
                # Successful voice feedback
                tts_success = TtsActionGoal()
                tts_success.goal.rawtext.text = "I have received your order. Ready for the next task!"
                tts_success.goal.rawtext.lang_id = "en_GB"
                tts_pub.publish(tts_success)
                rospy.sleep(2.5)

                shutdown_hook() 
                break
                
            else:
                # Step 6: Return to the starting point
                tts_msg = TtsActionGoal()
                tts_msg.goal.rawtext.text = "Okay. What do you need?"
                tts_msg.goal.rawtext.lang_id = "en_GB"
                tts_pub.publish(tts_msg)
                rospy.sleep(2.0)

        except Exception as e:
            print(f"Error occurred: {e}")
            print("Restarting input loop...\n")
        except KeyboardInterrupt:
            shutdown_hook()
            break

if __name__ == "__main__":
    main()

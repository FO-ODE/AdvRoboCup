import os
import sys
import torch
import whisper
import sounddevice as sd
import numpy as np
import wave
import random
import numpy as np

# 添加项目路径
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, "../src"))
sys.path.insert(0, project_root)

from minimind_chat_ros.model.model_minimind import MiniMindForCausalLM, MiniMindConfig
from minimind_chat_ros.model.model_lora import apply_lora, load_lora
from transformers import AutoTokenizer, TextStreamer

SAMPLERATE = 16000
DURATION = 5
WAV_FILE = "input.wav"


def setup_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def init_minimind():
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

    return model.eval().to("cpu"), tokenizer


def record_and_transcribe():
    print("🎤 Recording 5 seconds of audio...")
    audio = sd.rec(int(DURATION * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
    sd.wait()

    with wave.open(WAV_FILE, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLERATE)
        wf.writeframes(audio.tobytes())

    print("🧠 Transcribing with Whisper...")
    asr_model = whisper.load_model("base")
    result = asr_model.transcribe(WAV_FILE)
    return result["text"].strip()


def run_chat(model, tokenizer, text):
    setup_seed(random.randint(0, 2048))
    messages = [{"role": "user", "content": text}]

    prompt = tokenizer.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = tokenizer(prompt, return_tensors="pt", truncation=True).to("cpu")

    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)
    print("🤖️:", end='', flush=True)

    _ = model.generate(
        inputs["input_ids"],
        max_new_tokens=512,
        do_sample=True,
        attention_mask=inputs["attention_mask"],
        pad_token_id=tokenizer.pad_token_id,
        eos_token_id=tokenizer.eos_token_id,
        streamer=streamer,
        top_p=0.85,
        temperature=0.85
    )
    print("\n")


if __name__ == "__main__":
    model, tokenizer = init_minimind()

    while True:
        print("📢 Speak now (or Ctrl+C to exit)...")
        try:
            text = record_and_transcribe()
            print(f"🗣 You said: {text}")
            if text:
                run_chat(model, tokenizer, text)
        except KeyboardInterrupt:
            print("\n🛑 Exiting.")
            break

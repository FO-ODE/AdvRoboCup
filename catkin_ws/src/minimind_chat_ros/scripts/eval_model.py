import argparse
import random
import warnings
import numpy as np
import sys
import os
import torch

# 添加 minimind_chat_ros 路径
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, "../src"))
sys.path.insert(0, project_root)

from minimind_chat_ros.model.model_minimind import MiniMindForCausalLM, MiniMindConfig
from minimind_chat_ros.model.model_lora import apply_lora, load_lora
from transformers import AutoTokenizer, TextStreamer


warnings.filterwarnings('ignore')


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

    print(f'MiniMind模型参数量: {sum(p.numel() for p in model.parameters() if p.requires_grad) / 1e6:.2f}M')
    return model.eval().to("cpu"), tokenizer


def setup_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def main():
    model, tokenizer = init_model()
    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)

    messages = []
    print("🤖️ 输入 'exit' 可退出聊天")
    while True:
        prompt = input("👶: ")
        if prompt.strip().lower() == "exit":
            break
        setup_seed(random.randint(0, 2048))

        messages = []
        messages.append({"role": "user", "content": prompt})

        new_prompt = tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )

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


if __name__ == "__main__":
    main()

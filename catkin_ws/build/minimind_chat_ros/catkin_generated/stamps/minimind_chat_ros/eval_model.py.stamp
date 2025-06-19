import argparse
import random
import warnings
import numpy as np
import sys
import os
import torch

# # 添加 minimind_chat_ros 路径
# script_dir = os.path.dirname(__file__)
# project_root = os.path.abspath(os.path.join(script_dir, "../../minimind_chat_ros/src"))
# sys.path.insert(0, project_root)
# print(f"[DEBUG] minimind_chat_ros 路径添加为: {project_root}")
# 添加 ROS 消息路径
catkin_ws_path = os.path.expanduser("~/AdvRoboCup/catkin_ws")
devel_lib_path = os.path.join(catkin_ws_path, "devel", "lib", "python3", "dist-packages")
sys.path.insert(0, devel_lib_path)

# 添加 minimind_chat_ros 源码路径
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.abspath(os.path.join(script_dir, "..", "src", "minimind_chat_ros"))
sys.path.insert(0, package_dir)

import rospy
# from minimind_chat_ros.msg import ChatIntent
from msg import ChatIntent 
import re


# from minimind_chat_ros.model.model_minimind import MiniMindForCausalLM, MiniMindConfig
# from minimind_chat_ros.model.model_lora import apply_lora, load_lora
from model.model_minimind import MiniMindForCausalLM, MiniMindConfig
from model.model_lora import apply_lora, load_lora
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

def main():
    # 初始化 ROS 节点
    rospy.init_node("minimind_chat_node")
    intent_pub = rospy.Publisher("chat_intent", ChatIntent, queue_size=10)

    model, tokenizer = init_model()
    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)

    messages = []
    print("🤖️ Type 'exit' to quit the chat.")
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

        # 构造并发布 ChatIntent 消息
        action, object_, location = parse_intent_from_response(response)
        intent_msg = ChatIntent()
        intent_msg.action = action
        intent_msg.object = object_
        intent_msg.location = location
        intent_pub.publish(intent_msg)

if __name__ == "__main__":
    main()

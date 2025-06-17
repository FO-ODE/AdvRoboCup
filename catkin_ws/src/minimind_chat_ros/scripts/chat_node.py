#!/home/ubuntu/miniconda3/envs/adv_robocup/bin/python

import os
import sys
import rospy
import torch
import yaml
from std_msgs.msg import String
from minimind_chat_ros.msg import ChatIntent
from transformers import AutoTokenizer, TextStreamer

sys.path.append(os.path.join(os.path.dirname(__file__), '../model'))
from model_minimind import MiniMindConfig, MiniMindForCausalLM
from model_lora import apply_lora, load_lora


def parse_model_output(response_text):
    """
    从模型输出中提取 action、object、location 字段。
    格式如：
    action: pick
    object: "apple"
    location: "table"
    """
    action = object_ = location = "unknown"
    for line in response_text.splitlines():
        line = line.strip()
        if line.lower().startswith("action:"):
            action = line.split(":", 1)[1].strip().strip('"')
        elif line.lower().startswith("object:"):
            object_ = line.split(":", 1)[1].strip().strip('"')
        elif line.lower().startswith("location:"):
            location = line.split(":", 1)[1].strip().strip('"')
    return action, object_, location


class ChatNode:
    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # 从 YAML 加载参数
        config_file = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        model_path = config["model_path"]
        lora_path = config["lora_file"]
        base_ckpt = config["base_ckpt"]

        # 加载 tokenizer 和模型
        self.tokenizer = AutoTokenizer.from_pretrained(model_path)
        self.model = MiniMindForCausalLM(MiniMindConfig(hidden_size=512, num_hidden_layers=8))
        self.model.load_state_dict(torch.load(base_ckpt, map_location=self.device), strict=True)

        # 加载 LoRA 权重
        apply_lora(self.model)
        load_lora(self.model, lora_path)

        self.model.eval().to(self.device)
        self.streamer = TextStreamer(self.tokenizer, skip_prompt=True, skip_special_tokens=True)

        self.pub = rospy.Publisher("chat_response", String, queue_size=10)
        self.intent_pub = rospy.Publisher("chat_intent", ChatIntent, queue_size=10)
        rospy.Subscriber("chat_input", String, self.callback)

        rospy.loginfo("ChatNode initialized. Listening to /chat_input...")

    def callback(self, msg):
        prompt = self.tokenizer.bos_token + msg.data
        inputs = self.tokenizer(prompt, return_tensors="pt").to(self.device)
        generated = self.model.generate(
            inputs["input_ids"],
            max_new_tokens=512,
            do_sample=True,
            top_p=0.9,
            temperature=0.8
        )
        response = self.tokenizer.decode(
            generated[0][inputs["input_ids"].shape[1]:],
            skip_special_tokens=True
        )

        self.pub.publish(response)

        # 从模型响应中提取结构化意图
        action, obj, location = parse_model_output(response)
        intent_msg = ChatIntent()
        intent_msg.action = action
        intent_msg.object = obj
        intent_msg.location = location
        self.intent_pub.publish(intent_msg)

        rospy.loginfo(f"\n👤 用户: {msg.data}\n🤖 模型: {response}")
        rospy.loginfo(f"📦 意图解析结果 => action: {action}, object: {obj}, location: {location}")


if __name__ == '__main__':
    rospy.init_node('chat_node')
    ChatNode()
    rospy.spin()

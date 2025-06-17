#!/home/ubuntu/miniconda3/envs/adv_robocup/bin/python

import os
import sys
import rospy
import torch
import yaml
from std_msgs.msg import String
from transformers import AutoTokenizer, TextStreamer

sys.path.append(os.path.join(os.path.dirname(__file__), '../model'))
from model_minimind import MiniMindConfig, MiniMindForCausalLM
from model_lora import apply_lora, load_lora

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
        rospy.loginfo(f"\n👤 用户: {msg.data}\n🤖 模型: {response}\n")

if __name__ == '__main__':
    rospy.init_node('chat_node')
    ChatNode()
    rospy.spin()

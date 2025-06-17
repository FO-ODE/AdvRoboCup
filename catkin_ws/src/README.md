# MiniMind Chat ROS 项目说明

本项目通过 ROS 1 接入经过 LoRA 微调的语言模型，实现自然语言到结构化指令的解析，并发布到 ROS 话题中。

---

## Conda 环境配置

```bash
~/AdvRoboCup/catkin_ws/src/minimind_chat_ros/目录下：
conda env create -f adv_robocup_env.yaml
```
### 激活环境：
```bash
conda activate adv_robocup
```
---

## 创建 ROS 包

```bash
cd ~/AdvRoboCup/catkin_ws/src
catkin_create_pkg minimind_chat_ros rospy std_msgs
```

将代码与模型文件放入如下结构：

```
minimind_chat_ros/
├── scripts/
│   └── chat_node.py             # 主节点 Python 脚本（需要 chmod +x）
├── model/                       # 自定义 tokenizer 与模型配置
│   ├── __init__.py
│   ├── model_minimind.py
│   ├── model_lora.py
│   ├── tokenizer.json
│   ├── tokenizer_config.json
├── policy/                      # 微调模型权重文件
│   ├── lora_robocup_512.pth
│   ├── pretrain_512.pth
```

---

## 添加执行权限

```bash
chmod +x ~/AdvRoboCup/catkin_ws/src/minimind_chat_ros/scripts/chat_node.py
```

---

## 编译 ROS 工程

```bash
cd ~/AdvRoboCup/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 使用 Python 启动节点（替代 roslaunch）

由于我们直接传参失败，可改为 YAML 配置方式：

1. 创建配置文件：

`config/chat_config.yaml` 内容如下：

```yaml
model_path: "/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/model"
lora_file: "/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/policy/lora_robocup_512.pth"
base_ckpt: "/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/policy/pretrain_512.pth"
```

2. 修改 `chat_node.py` 中参数加载方式：

```python
import yaml
config_file = os.path.join(os.path.dirname(__file__), "../config/chat_config.yaml")
with open(config_file, "r") as f:
    config = yaml.safe_load(f)

model_path = config["model_path"]
lora_path = config["lora_file"]
base_ckpt = config["base_ckpt"]
```

3. 启动节点：

```bash
python ~/AdvRoboCup/catkin_ws/src/minimind_chat_ros/scripts/chat_node.py
```
## **注意** 
构建 ROS 工作空间时，请 不要在 Conda 环境中执行 catkin_make，否则可能会因为 Python 路径冲突导致构建失败或找不到依赖
推荐流程如下：

1. 退出 Conda 环境，在系统默认环境下运行 catkin_make 构建工作空间；

2. 构建成功后，再重新进入 Conda 环境；

3. 在 Conda 环境中执行 source ~/AdvRoboCup/catkin_ws/devel/setup.bash；

4. 最后运行聊天节点脚本 chat_node.py
---

## 输入输出接口说明

### 输入话题（订阅）：

- 话题名：`/chat_input`
- 消息类型：`std_msgs/String`
- 示例命令：

```bash
rostopic pub /chat_input std_msgs/String "data: 'pick the apple on the table'"
```

---

### 输出话题（发布）：

- 话题名：`/chat_intent`
- 消息类型：`ChatIntent.msg`（自定义）

```msg
string action
string object
string location
```

- 示例查看：

```bash
rostopic echo /chat_intent
```

---

## 检查点列表

- 模型位置：
  - `/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/model/`
- 策略位置：
  - `/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/policy/`
- 主程序路径：
  - `/home/ubuntu/AdvRoboCup/catkin_ws/src/minimind_chat_ros/scripts/chat_node.py`

---

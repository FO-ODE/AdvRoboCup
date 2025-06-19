# MiniMind Chat ROS 项目说明

本项目基于 LoRA 微调的语言模型，通过 ROS 1 框架解析自然语言指令，并发布结构化意图信息至 ROS 话题 `/chat_intent`。适用于 RoboCup\@Home 等机器人任务中的语义理解模块。

---

## 环境配置

### 1. 创建 Conda 环境并安装依赖

```bash
conda create -n minimind python=3.10 -y
conda activate minimind
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 构建 ROS 工作空间

### 1. 编译消息类型

```bash
cd ~/AdvRoboCup/catkin_ws
catkin_make
source devel/setup.bash
```

**注意：** 请确保构建时不处于 Conda 环境中，以避免 Python 路径冲突。推荐流程：

1. 退出 Conda 环境后执行 `catkin_make`；
2. 构建完成后再进入 `minimind` 环境；
3. 执行 `source devel/setup.bash` 再运行脚本。

---

## 启动模型脚本

```bash
python src/minimind_chat_ros/scripts/eval_model.py
```

该脚本将加载 LoRA 微调模型，并开始意图解析和发布。

---

## 项目结构说明

```
minimind_chat_ros/
├── CMakeLists.txt                 
├── package.xml                   
├── msg/
│   └── ChatIntent.msg            # 自定义意图消息
├── scripts/
│   └── eval_model.py             # 启动脚本（需 Python 执行）
├── src/
│   └── minimind_chat_ros/
│       ├── dataset/              # 数据集模块（可选）
│       ├── model/                # 模型定义与权重
│       │   ├── model_minimind.py
│       │   ├── model_lora.py
│       │   ├── tokenizer.json
│       │   └── tokenizer_config.json
│       └── out/
│           ├── full_sft_512.pth          # 完整微调模型（可选）
│           └── lora/
│               └── lora_medical_512.pth  # LoRA 参数
```

---

## 发布话题 `/chat_intent`

- **话题名称**：`/chat_intent`
- **消息类型**：`minimind_chat_ros/ChatIntent.msg`

```msg
string action
string object
string location
```

### 示例查看输出

```bash
rostopic echo /chat_intent
```

---

## 检查点文件路径

- 模型目录：
  - `src/minimind_chat_ros/model/`
- 权重文件：
  - `src/minimind_chat_ros/out/lora/lora_medical_512.pth`
- 启动脚本：
  - `src/minimind_chat_ros/scripts/eval_model.py`

---
## Git LFS
1. 安装
```bash
sudo apt install git-lfs
```

2. 执行一次初始化
```bash
git lfs install
```
3. 拉取 LFS 管理的 .pth 文件
```bash
cd ~/AdvRoboCup  # 或你的仓库根目录
git lfs pull
```


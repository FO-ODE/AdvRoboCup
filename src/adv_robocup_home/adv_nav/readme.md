# Head Scanner Node

控制 TIAGo 机器人头部左右扫视。

## 功能

- 左右扫视：±60度
- 模式：单次、持续、归位
- 实时控制：话题命令

## 使用

### 启动

- 持续扫视：`rosrun adv_nav head_scan.py`
- 单次扫视：`rosrun adv_nav head_scan.py _mode:=single`
- 归位：`rosrun adv_nav head_scan.py _mode:=center`

### 控制

- 停止：`rostopic pub /head_scan_command std_msgs/String "data: 'stop'"`

## 配置

- `scan_range`：扫视角度
- `scan_speed`：移动速度
- `pause_time`：停留时间

## ROS 接口

### 话题

- `/head_scan_command`：控制命令

### Action 服务

- `/head_controller/follow_joint_trajectory`：控制头部

## 注意

- 启动时归位
- `stop` 保持当前位置
- `center` 手动归位
- 确保头部控制器运行

---

## Person Follower Node

## 概述

`person_follower.py` 节点旨在使 TIAGo 机器人跟随人。它订阅人的位姿信息，并向 `move_base` 发送目标点，使机器人保持与人的预设距离。

## 功能

- 订阅 `/person_pose` 话题，获取人的位姿信息。
- 将目标点发布到 `/move_base_simple/goal` 话题，控制机器人移动。
- 使用 TF2 转换坐标系，确保目标点在地图坐标系下正确。
- 维护与人的目标距离，并在机器人到达目标点后停止移动。
- 异常处理，包括 TF 查找失败和其他错误。

## 用法

1. 确保已安装所有依赖项（例如 `tf2_ros`、`geometry_msgs`）。
2. 运行该节点：

    ```bash
    rosrun adv_nav person_follower.py
    ```

3. 确保有一个发布 `/person_pose` 话题的节点正在运行，提供人的位姿信息。

## 话题

### 订阅

- `/person_pose` (geometry_msgs/PoseStamped): 人的位姿信息，相对于机器人的坐标系。

### 发布

- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 发送给 `move_base` 的目标点。

## 参数

- `follow_distance` (float, default: 1.3): 机器人与人保持的目标距离（米）。

- `tolerance` (float, default: 0.3): 机器人到达目标点的容忍度（米）。

## 许可证

MIT 许可证

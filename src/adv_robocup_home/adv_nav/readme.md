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
- 确保头部控制器
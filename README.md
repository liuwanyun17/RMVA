## 使用说明
通过摄像头在Gazebo的仿真世界进行识别，装甲板中心数字排序，弹丸击打等过程

统一内容：

1.打开Gazebo摄像头仿真

不同内容：

1.打开不同模型

2.调节模型位置

3.打开不同可执行文件

## 文件结构
```plain
RMVA/ 
├── CMakeLists.txt 
├── package.xml 
├── launch/ 
│ ├── vision.launch # 启动视觉节点 
 
├── src/ 

│ ├── sphere_vision_node.cpp # 球形识别主程序 
│ ├── rec_vision_node.cpp # 矩形识别主程序 
│ ├── armor_vision_node.cpp # 装甲板识别主程序 
│ ├── multi_armor_vision_node.cpp # 多目标装甲板识别主程序
│ ├── shooter_node.cpp # 弹丸击打控制程序 
| |__ race_stage_subscriber.cpp #比赛阶段切换订阅者

├── config/ 
│ └── params.yaml # 参数配置⽂件 
├── digit_templates/ # 模板匹配使用的数字图片
├── results/ # 测试截图、⽇志等 
└── README.md # 使⽤说明、依赖项、算法原理（含运⾏指令）
```

## 依赖项
player_pkg

target_model_pkg

referee_pkg
## 算法原理（含运行指令）

## 运行摄像头仿真
```bash
ros2 launch camera_sim_pkg camera.launch.py
```
## 开启装甲板（模型这里统一用装甲板代替）
```bash
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_1.sdf model_name:=armor_red_1
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_2.sdf model_name:=armor_red_2
```
具体打开的模型根据识别内容需要
## 修改朝向和位置，使装甲板正面朝摄像头且摄像头能够识别完整的装甲板（y一般改成7.0就可以）（这个可以位置和朝向双修改，效果比较好）
```bash
ros2 topic pub /pose geometry_msgs/msg/Pose "{position: {x: 1.0, y: 7.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"
```
## 然后旋转通过改变data后面的数字实现不同状态的旋转进行动态识别
```bash
ros2 topic pub /type std_msgs/msg/Int32 "{data: 1}"
```
## 运行
```bash
#球形识别
ros2 run player_pkg sphere_vision_node
#矩形识别
ros2 run player_pkg rec_vision_node
#装甲板识别
ros2 run player_pkg armor_vision_node
#多目标装甲板识别
ros2 run player_pkg multi_armor_vision_node
#弹丸击打程序
ros2 run player_pkg armor_shooter_node
```


# DRL-PID
# 基于深度强化学习的自适应PID控制器方法（应用于智能车循迹任务）

summit_description 为该工程包括的ros包，包括了智能车模型和gazebo环境模型；my_ground_plane 为跑道的模型信息，需要放在.gazebo/models下；

工程复现步骤：
1. 运行智能车ROS节点：roslaunch summit_description summit.launch 
2. 运行 env_feedback.py (此Python脚本用于摄像头检测轨迹信息)
3. 运行 main.py (主文件)

在第二步中，Python环境为2.7；第三步中，Python环境为3.8,且需要手动把Python的依赖包的路径添加到ros自带的python路径前面：export PYTHONPATH=/media/superd/download/Linux/miniconda3/envs/pytorch/lib/python3.8/site-packages:$PYTHONPATH

第三步代码的依赖库：
torch；numpy; catkin-pkg; matplotlib; rospkg; setuptools; 


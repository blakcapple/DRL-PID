# DRL-PID

## Intro

It's a framework that focuses on the dynamic adjustment of the PID parameters of the controlled objects using **Deep Reinforcement Learning Algorithm**, which in this project, I used **Soft Actor Critic** (SAC) as the basic DRL algorithm and achieved great results. 

Specifically, this project is using DRL-PID framework in the **auto driving scenario** (track following), but this framework can expand to any scene that needs dynamically tuning PID parameters without human intervention. The **main advantage** of this framework in my opinion is it can **search the optimal PID tuning policy automatically**, especially when too many PID parameters are coupled together, and manual adjustment is difficult. But it also suffers from the unexplainable deep learning procedure and can not provide the theory guarantee.   

**It's my bachelor graduation project and I hope the open source code can benefit  the future research in the relevant domains.** 

**If you find this project helpful, star on it :star:**

## Method 

#### Overview

The training framework: 

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/framework.png)

The execution framework:

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/structure.png)

#### Line Detection 

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/searching%20algorithm.png)

#### ROS Node

RL training is implemented in the **ROS** and **gazebo** platform

The training system operates using the ROS topic communication mechanism. 

The overall system mainly includes **environment detection node** and **tracking control node**. 

Environment detection node: **Subscribe to the topics released by the camera**, pre-process the images detected by the camera, and obtain state information; at the same time, judge whether the smart car deviates from the track by judging the length of the left and right boundary point sets obtained by the image search algorithm, and store these information as ROS messages to post on the Environment Feedback topic.

tracking control node: Subscribe to the environmental feedback topic and the topic **odom** released by GAZEBO that contains the pose information of the car; **output angular velocity information and linear velocity** information to the topic **cmd_vel**, and **cmd_vel** controls the movement of the smart car simulation model.
Finally, GAZEBO subscribes to the **cmd_vel** topic to visualize the motion of the smart car simulation model in real-time in the tracking environment , eventually form a **complete control closed-loop structure**.

## Installation   

* ROS (16.04 or 20.04 has been tested alright)
* Gazebo 

* Pytorch 

## Quick Start

**summit_description** contains the ROS package needed in this project, install this in your own ROS workspace first and add it to ROS path. 

**launch the environment**

```
roslaunch summit_description summit.launch
```

**env_feedback.py** contains the algorithm that detects the track using camera and processes the original environment detection into the state information.

**launch the env detection node**

```
python env_feedback.py
```

after the preparation steps, you can launch the main node to start training.

**start training**

```
python main.py
```

## Code Structure

**my_ground_plane**

the path model used in the project, put this into the .gazebo/model folder. You can change the map by replacing the **MyImage.png** in the /materials/textures

**summit description**

* contain robot description, necessary ros message, ros service file and launch file 

**DRL-PID** 

* main.py : main file for training  
* model_testing.py : test trained model 
* sac_torch.py : SAC algorithm
* network.py: network definition 
* env_feedback.py: the env pre-process file 
* line_follower.py: the environment definition 

## Experiment

**map_0** 

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/map_0.png)

**episode reward**

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/score_plot.png)

**episode success rate**

![image](https://raw.githubusercontent.com/blakcapple/DRL-PID/main/Image/success_rate.png)

## Reference 

[A Self-adaptive SAC-PID Control Approach based on Reinforcement Learning for Mobile Robots](https://arxiv.org/pdf/2103.10686.pdf) (this project is highly based on this paper)

 






























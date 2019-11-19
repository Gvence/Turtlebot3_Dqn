#coding:utf-8
#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import os
import numpy as np
from math import pi
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn


class Env():
    def __init__(self, action_size):
        self.heading = 0
        self.yaw = 0
        self.last_heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.initBurger = True
        self.burgerName = 'turtlebot3_burger'
        self.get_goalbox = False
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.reset_burger = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.respawn = Respawn()
        self.get_goal_step = 0

    def resetBurger(self):
        state = ModelState()
        burgerPosition = Pose()
        position_check = True
        last_burger_x = self.position.x
        last_burger_y = self.position.y
        while position_check:
            burgerPosition.position.x = random.randrange(-12, 13) / 10.0
            burgerPosition.position.y = random.randrange(-12, 13) / 10.0
            yaw = 1.0*random.randrange(1,314)/100
            orientation = quaternion_from_euler(0,0,yaw)
            burgerPosition.orientation.x = orientation[0]
            burgerPosition.orientation.y = orientation[1]
            burgerPosition.orientation.z = orientation[2]
            burgerPosition.orientation.w = orientation[3]

            if abs(burgerPosition.position.x - self.respawn.obstacle_1[0]) <= 0.4 \
                    and abs(burgerPosition.position.y - self.respawn.obstacle_1[1]) <= 0.4:
                position_check = True
            elif abs(burgerPosition.position.x - self.respawn.obstacle_2[0]) <= 0.4 \
                    and abs(burgerPosition.position.y - self.respawn.obstacle_2[1]) <= 0.4:
                position_check = True
            elif abs(burgerPosition.position.x - self.respawn.obstacle_3[0]) <= 0.4 \
                    and abs(burgerPosition.position.y - self.respawn.obstacle_3[1]) <= 0.4:
                position_check = True
            elif abs(burgerPosition.position.x- self.respawn.obstacle_4[0]) <= 0.4 \
                    and abs(burgerPosition.position.y - self.respawn.obstacle_4[1]) <= 0.4:
                position_check = True
            elif abs(burgerPosition.position.x - self.respawn.goal_position.position.x) <= 0.6 \
                    and abs(burgerPosition.position.y - self.respawn.goal_position.position.y) <= 0.6:
                position_check = True
            elif abs(burgerPosition.position.x - last_burger_x) < 1 and abs(
                        burgerPosition.position.y - last_burger_y) < 1:
                position_check = True
            else:
                position_check = False
        state.pose = burgerPosition
        state.model_name = 'turtlebot3_burger'
        self.reset_burger.publish(state)
        rospy.loginfo('Burger position : %.1f, %.1f, Yaw: %.1f'%(burgerPosition.position.x, burgerPosition.position.y, yaw))
        return burgerPosition.position.x, burgerPosition.position.y

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.respawn.goal_position.position.x
                                         - self.position.x,
                                         self.respawn.goal_position.position.y
                                         - self.position.y), 8)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        goal_angle = math.atan2(self.respawn.goal_position.position.y - self.position.y, self.respawn.goal_position.position.y - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi
        self.yaw = round(yaw, 8) # 直接使用yaw作为姿态回报

    def getState(self, scan):
        # 每个度数一个距离信息总共３６０个
        scan_range = []
        heading = self.heading
        min_range = 0.12
        done = 0

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 8)
        self.obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = 1

        self.goal_distance = self.getGoalDistace()
        self.current_distance = obstacle_min_range
        #IG_position = self.position.x, self.position.y, self.respawn.goal_position.position.x, self.respawn.goal_position.position.y

        if self.goal_distance < 0.56:
            self.get_goalbox = True
        for i in [heading, obstacle_min_range, self.obstacle_angle]:
            scan_range.append(i)
        return scan_range, self.goal_distance,done

    def setReward(self, state, done, action, steps):
        # yaw_reward = []
        #
        # for i in range(5):
        #     angle = -math.pi / 4 + self.heading + (math.pi / 8 * i) + math.pi / 2
        #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #     yaw_reward.append(tr)
            # 每次转动朝向都会产生负回报,因此机器人会倾向于走直线
            # 单次转向不超过pi

        distance_rate =  3.0 /self.goal_distance - 1.0/self.current_distance
        #distance_rate = 2**(self.current_distance/self.goal_distance)



        #- math.pow(((steps - self.get_goal_step)*0.02),2)),8) step压力计算，越长时间无碰撞回报越低
        #如果到达BOX,step压力会置零, 0.002 = 1 / 50, 当50步一直没有找到box就开始给大于-1的反馈,反馈成平方级增长
        # -1/5*(yaw_reward[action] + 0.22), 某一步转向角越接近45度扣分越大,越接近0或90度扣分越少, 0或90度的扣分为1/5 * (pi/4 + 0.22) = 0.2
        # 45度扣分为1/0.22*5= 1
        # 我们希望小车在30步内找到目标,所以30步开始回报要大于-1,这样便于网络判断本次运行的有效性,所以: -1 * 50 + reward >= 1
        # 所以找到目标的reward = 50
        angleR = (1.0/2)**(math.fabs(self.heading - self.obstacle_angle*math.pi/360))
        headR =  math.fabs(self.last_heading - self.heading)*1.5
        disR = distance_rate
        stepR = math.pow(((steps - self.get_goal_step)*0.01),2)
        reward = round(angleR - headR + disR -stepR ,8)
        #reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)
        #print 'angel:', angleR, 'heading:', -headR, 'disr:', disR, 'step:', -stepR, 'Reward:', reward
        #reward = 0
        if done:
            rospy.loginfo("Collision!!")
            # 撞墙就-50
            reward = -5
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            # 找到东西就有奖励
            reward = 10
            self.pub_cmd_vel.publish(Twist())
            self.respawn.getPositionGoal(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def step(self, action,steps):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        # 运动前记录头朝向
        self.last_heading = self.heading
        self.pub_cmd_vel.publish(vel_cmd)
        # 运动后更新头朝向
        self.heading = self.yaw
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, goal_distance,done = self.getState(data)
        #如果到达BOX记录到达时的step，用于清零step压力
        if self.get_goalbox:
            self.get_goal_step = steps

        reward = self.setReward(state, done, action,steps)
        if done :
            self.get_goal_step = 0
        return state, reward, goal_distance, done

    def reset(self):
        if self.initBurger:
            self.initBurger = False
            rospy.wait_for_service('gazebo/reset_simulation')
            try:
                self.reset_proxy()
            except (rospy.ServiceException) as e:
                print("gazebo/reset_simulation service call failed")
        else:
            self.last_heading = self.heading
            self.resetBurger()
            self.heading = self.yaw
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.respawn.getPositionGoal()
            self.initGoal = False
        # else:
        #     self.respawn.getPositionGoal(delete=True)
        self.goal_distance = self.getGoalDistace()
        state, goal_distance,done = self.getState(data)

        return state,goal_distance
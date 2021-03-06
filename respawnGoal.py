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
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self):
        self.modelGoalPath = os.path.dirname(os.path.realpath(__file__))
        self.modelGoalPath = self.modelGoalPath.replace('turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelGoalPath, 'r')
        self.modelGoal = self.f.read()
        #
        # self.modelBurgerPath = os.path.dirname(os.path.realpath(__file__))
        # self.modelBurgerPath = self.modelBurgerPath.replace('turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
        #                                         'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')
        # self.j = open(self.modelBurgerPath, 'r')
        # self.modelBurger = self.j.read()
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.stage = 2  # rospy.get_param('/stage_number')
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelNameGoal = 'goal_box'
        self.modelNameBurger = 'burger_name'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_modelGoal = False
        self.check_modelBurger = False
        self.index = 0

    def getOdometry(self, odom):
        self.burger_position = odom.pose.pose.position
        
    def checkModel(self, model):
        self.check_modelGoal = False
        # self.check_modelBurger = False
        for i in range(len(model.name)):
            if model.name[i] == "goal_box":
                self.check_modelGoal = True
            # elif model.name[i] == "turtlebot3_burger":
            #     self.check_modelBurger = True

    def respawnModelGoal(self, ):
        while True:
            if not self.check_modelGoal:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelNameGoal, self.modelGoal, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    # def respawnModelBurger(self, ):
    #     while True:
    #         if not self.check_modelBurger:
    #             rospy.wait_for_service('gazebo/spawn_sdf_model')
    #             spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    #             spawn_model_prox(self.modelNameBurger, self.modelBurger, 'robotos_name_space', self.goal_position, "world")
    #             rospy.loginfo("Goal position : %.1f, %.1f", self.burger_position.position.x,
    #                           self.burger_position.position.y)
    #             break
    #         else:
    #             pass

    def deleteModelGoal(self, ):
        while True:
            if self.check_modelGoal:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelNameGoal)
                break
            else:
                pass

    # def deleteModelBurger(self, ):
    #     while True:
    #         if self.check_modelBurger:
    #             rospy.wait_for_service('gazebo/delete_model')
    #             del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    #             del_model_prox(self.modelNameBurger)
    #             break
    #         else:
    #             pass

    def getPositionGoal(self, position_check=True, delete=False):
        if delete:
            self.deleteModelGoal()

        # if self.stage != 4:
        while position_check:
            goal_x = random.randrange(-12, 13) / 10.0
            goal_y = random.randrange(-12, 13) / 10.0
            if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.burger_position.x) <= 0.6 and abs(goal_y - self.burger_position.y) <= 0.6:
                position_check = True
            else:
                position_check = False

            if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                position_check = True

            self.goal_position.position.x = goal_x
            self.goal_position.position.y = goal_y

        # else:
        #     while position_check:
        #         goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
        #         goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]
        #
        #         self.index = random.randrange(0, 13)
        #         print(self.index, self.last_index)
        #         if self.last_index == self.index:
        #             position_check = True
        #         else:
        #             self.last_index = self.index
        #             position_check = False
        #
        #         self.goal_position.position.x = goal_x_list[self.index]
        #         self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModelGoal()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y
        return self.goal_position.position.x, self.goal_position.position.y
    # def getPositionBurger(self, position_check=True, delete=False):
    #     if delete:
    #         self.deleteModelBurger()
    #
    #     # if self.stage != 4:
    #     while position_check:
    #         burger_x = random.randrange(-12, 13) / 10.0
    #         burger_y = random.randrange(-12, 13) / 10.0
    #         if abs(burger_x - self.obstacle_1[0]) <= 0.4 and abs(burger_y - self.obstacle_1[1]) <= 0.4:
    #             position_check = True
    #         elif abs(burger_x - self.obstacle_2[0]) <= 0.4 and abs(burger_y - self.obstacle_2[1]) <= 0.4:
    #             position_check = True
    #         elif abs(burger_x - self.obstacle_3[0]) <= 0.4 and abs(burger_y - self.obstacle_3[1]) <= 0.4:
    #             position_check = True
    #         elif abs(burger_x - self.obstacle_4[0]) <= 0.4 and abs(burger_y - self.obstacle_4[1]) <= 0.4:
    #             position_check = True
    #         elif abs(burger_x - self.goal_position.position.x) <= 0.6 and abs(burger_y - self.goal_position.position.y) <= 0.6:
    #             position_check = True
    #         else:
    #             position_check = False
    #
    #         if abs(burger_x - self.last_burger_x) < 1 and abs(burger_y - self.last_burger_y) < 1:
    #             position_check = True
    #
    #         self.burger_position.position.x = burger_x
    #         self.burger_position.position.y = burger_y

        # else:
        #     while position_check:
        #         goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
        #         goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]
        #
        #         self.index = random.randrange(0, 13)
        #         print(self.index, self.last_index)
        #         if self.last_index == self.index:
        #             position_check = True
        #         else:
        #             self.last_index = self.index
        #             position_check = False
        #
        #         self.goal_position.position.x = goal_x_list[self.index]
        #         self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModelBurger()

        self.last_burger_x = self.burger_position.position.x
        self.last_burger_y = self.burger_position.position.y


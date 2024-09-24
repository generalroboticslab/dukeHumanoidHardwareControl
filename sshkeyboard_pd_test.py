import time
from sshkeyboard import listen_keyboard
import asyncio
from publisher import DataPublisher
import random
import sys
import numpy as np
pi = np.pi

from contextlib import contextmanager



def set_zero():
    print("reseting to zeroth position")
    change_pos_dof_target(target=np.zeros(10),action_is_on=np.ones(10,dtype=bool))


@contextmanager
def conditionally_publish(should_set_zero=True):
    try:
        change_publish_state(should_publish=True)  # Assuming this enables motor control
        yield
    finally:
        change_publish_state(should_publish=False)  # Assuming this enables motor control
        if should_set_zero:
            set_zero()


data_publisher = DataPublisher('udp://localhost:9871',broadcast=True)
# data_publisher2 = DataPublisher('udp://localhost:9872',broadcast=True)

def publish(data):
    data_publisher.publish(data)
    # data_publisher2.publish(data)


def change_publish_state(should_publish):
    for k in range(10):
        publish({"should_publish":should_publish})
        time.sleep(0.01)

default_action_is_on = np.ones(10)
def change_pos_dof_target(target,action_is_on=default_action_is_on):
    data = {
            "dof_pos_target":target,
            "action_is_on":action_is_on,
        }
    publish(data)

dt = 1/400

# REFERENCE
# self.min_limit = np.array([-pi/3, -pi/6, -pi/9,    0,   -pi/5, -pi/6, -pi/2, -pi/5, -pi/2, -pi/5])
# self.max_limit = np.array([pi/6,   pi/2,  pi/5, pi/2,    pi/5,  pi/3,  pi/6,  pi/9,     0,  pi/5])  

def sweep(t, a, b, freq_start, freq_end):

    x = np.arange(0, t, dt)

    frequencies = np.linspace(freq_start, freq_end, x.size)
    

    y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)

    action = np.ones_like(x)

    off_ratio=0
    hz=50
    cluster_repeat = int((1/dt)*(1/hz)) #amount of points to make up x hz
    tentative_action_is_on = np.repeat(np.random.rand(len(action)//cluster_repeat)>off_ratio,cluster_repeat)
    action[:len(tentative_action_is_on)] = tentative_action_is_on
    
    return np.concatenate((y, np.flip(y))), np.concatenate((action, np.flip(action)))

def indvidual_motor_test_passive(key):
    if key == "0":
        with conditionally_publish(should_set_zero=True):
            y, action = sweep(t=12, a=-pi/5, b=pi/5, freq_start=0.5, freq_end=3.5)
            print("motor " + key + " is moving")
            for i, act in zip(y, action): 
                change_pos_dof_target(target=[i, 0, 0, 0, 0, -i, 0, 0, 0, 0], action_is_on=[act, 1, 1, 1, 1, act, 1, 1, 1, 1])
                time.sleep(dt)

    if key == "1":
        with conditionally_publish(should_set_zero=True):
            y, action = sweep(t=12, a=-pi/5, b=pi/5, freq_start=0.5, freq_end=3)
            print("motor " + key + " is moving")
            for i, act in zip(y, action): 
                change_pos_dof_target(target=[0, i, 0, 0, 0, 0, i, 0, 0, 0], action_is_on=[1, act, 1, 1, 1, 1, act, 1, 1, 1])
                time.sleep(dt)

    if key == "2":
        with conditionally_publish(should_set_zero=True):
            y, action = sweep(t=12, a=-pi/5.5, b=pi/5.5, freq_start=0.5, freq_end=2.5)
            print("motor " + key + " is moving")
            # for i, act in zip(y, action): 
            #     change_pos_dof_target(target=[0, 0, i, 0, 0, 0, 0, i, 0, 0], action_is_on=[1, 1, act, 1, 1, 1, 1, act, 1, 1])
            #     time.sleep(dt)
                
            for i, act in zip(y, action): 
                change_pos_dof_target(target=[0, 0, i, 0, 0, 0, 0, -pi/4.5, 0, 0], action_is_on=[1, 1, act, 1, 1, 1, 1, 1, 1, 1])
                time.sleep(dt)
                
            # for i, act in zip(y, action): 
            #     change_pos_dof_target(target=[0, 0, pi/4.5, 0, 0, 0, 0, i, 0, 0], action_is_on=[1, 1, 1, 1, 1, 1, 1, act, 1, 1])
            #     time.sleep(dt)
                
    if key == "3":
        with conditionally_publish(should_set_zero=True):
            y, action = sweep(t=12, a=0, b=pi/2, freq_start=0.5, freq_end=3)
            print("motor " + key + " is moving")
            for i, act in zip(y, action): 
                change_pos_dof_target(target=[0, 0, 0, i, 0, 0, 0, 0, -i, 0], action_is_on=[1, 1, 1, act, 1, 1, 1, 1, act, 1])
                time.sleep(dt)

    if key == "4":
        with conditionally_publish(should_set_zero=True):
            y, action = sweep(t=12, a=pi/5.5, b=-pi/5.5, freq_start=0.5, freq_end=3)
            print("motor " + key + " is moving")
            for i, act in zip(y, action): 
                change_pos_dof_target(target=[0, 0, 0, 0, i, 0, 0, 0, 0, -i], action_is_on=[1, 1, 1, 1, act, 1, 1, 1, 1, act])
                time.sleep(dt)
    if key=='q':
        with conditionally_publish(should_set_zero=True):
            y3 = sweep(5,0,pi/4,1,3)
            y4 = sweep(5,pi/5.5,-pi/5.5,1,4)
            print("motor " +key+ " is moving")
            for i,j in zip(y3,y4):
                change_pos_dof_target(target=[0, 0, 0, i, j, 0, 0, 0, -i, -j])
                time.sleep(dt)
    if key=='w':
        with conditionally_publish(should_set_zero=True):
            y2 = sweep(5,-pi/15,pi/15,1.5,2.5)
            y3 = sweep(5,0,pi/4,1,3)
            y4 = sweep(5,pi/5.5,-pi/5.5,1,4)
            print("motor " +key+ " is moving")
            for y2i,y3i,y4i in zip(y2,y3,y4):
                change_pos_dof_target(target=[0, 0, y2i, y3i, y4i, 0, 0, y2i, -y3i, -y4i])
                time.sleep(dt)
    if key=='e':
        with conditionally_publish(should_set_zero=True):
            y1 = sweep(5,-pi/15,pi/15,1,3)
            y2 = sweep(5,-pi/15,pi/15,1.5,2.5)
            y3 = sweep(5,0,pi/4,1,3)
            y4 = sweep(5,pi/5.5,-pi/5.5,1,4)
            print("motor " +key+ " is moving")
            for y1i, y2i,y3i,y4i in zip(y1,y2,y3,y4):
                change_pos_dof_target(target=[0, y1i, y2i, y3i, y4i, 0, y1i, y2i, -y3i, -y4i])
                time.sleep(dt)
    if key=='r':
        with conditionally_publish(should_set_zero=True):
            y0,a0 = sweep(30,-pi/5,pi/5,0.5,2.5)
            y1,a1 = sweep(30,-pi/5,pi/5,0.5,2.5)
            y2,a2 = sweep(30,-pi/8,pi/8,0.5,2.5)
            y3,a3 = sweep(30,0,pi/2,0.5,2.5)
            y4,a4 = sweep(30,pi/5.5,-pi/5.5,0.5,2.5)
            print("motor " +key+ " is moving")
            for y0i, y1i, y2i,y3i, y4i, a0i, a1i, a2i, a3i, a4i in zip(y0, y1,y2,y3,y4,a0,a1,a2,a3,a4):
                change_pos_dof_target(target=[y0i, y1i, y2i, y3i, y4i, y0i, y1i, y2i, -y3i, -y4i], 
                                      action_is_on=[a0i, a1i, a2i, a3i, a4i, a0i, a1i, a2i, a3i, a4i])
                time.sleep(dt)
    if key == "9":
        set_zero()


def walking_test(key):
    if key == "0":
        import os
        import pandas as pd

        # Read trajectory data from CSV file
        output_folder = '/home/grl/repo/legged_env/trajectories'
        input_file = os.path.join(output_folder, 'walking_trajectory.csv')
        trajectory_data = pd.read_csv(input_file)

        positions = []
        for i in range(len(trajectory_data)):
            positions.append([float(trajectory_data['0_hip_L'][i]), float(trajectory_data['1_hip_L'][i]), float(trajectory_data['2_hip_L'][i]),
                              float(trajectory_data['3_knee_L'][i]), float(trajectory_data['4_foot_L'][i]),
                              float(trajectory_data['5_hip_R'][i]), float(trajectory_data['6_hip_R'][i]), float(trajectory_data['7_hip_R'][i]),
                              float(trajectory_data['8_knee_R'][i]), float(trajectory_data['9_foot_R'][i])])
            print(positions[-1])
        if key == '0':

            dt = 0.01
        while(True):
            for pos in positions:
                change_pos_dof_target(key='0',target=pos)
                time.sleep(dt)    

def press(key):

    # indvidual_motor_test(key)

    indvidual_motor_test_passive(key)

    # walking_test(key)

    # if key=='9':
    #     data = {
    #         "reset":True,
    #         "id":random.randint(-sys.maxsize, sys.maxsize)
    #     }
    #     # reset
    #     publish(data)
    #     print("9 pressed")
        
listen_keyboard(on_press=press,delay_second_char=0.001,delay_other_chars=0.005,sleep=0.0001)


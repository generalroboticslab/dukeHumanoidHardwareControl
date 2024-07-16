import time
from sshkeyboard import listen_keyboard
import asyncio
from publisher import DataPublisher
import random
import sys
import numpy as np
pi = np.pi

data_publisher = DataPublisher('udp://localhost:9871')
data_publisher2 = DataPublisher('udp://localhost:9872')

def publish(data):
    data_publisher.publish(data)
    data_publisher2.publish(data)


def change_pos_dof_target(key,target):
    data = {
            "dof_pos_target":target,
            "id":random.randint(-sys.maxsize, sys.maxsize)
        }
    publish(data)
    print(f"{key} pressed")

# REFERENCE
# self.min_limit = np.array([-pi/3, -pi/6, -pi/9,    0,   -pi/5, -pi/6, -pi/2, -pi/5, -pi/2, -pi/5])
# self.max_limit = np.array([pi/6,   pi/2,  pi/5, pi/2,    pi/5,  pi/3,  pi/6,  pi/9,     0,  pi/5])  

def motor_zero_test(key):
    if key=='1':
        change_pos_dof_target(key='4',target=[-pi/4, 0, 0, 0, 0, pi/4, 0, 0, 0, 0])

    if key=='2':
        change_pos_dof_target(key='3',target=[-pi/8, 0, 0, 0, 0, pi/8, 0, 0, 0, 0])
    if key=='3':
        change_pos_dof_target(key='1',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='4':
        change_pos_dof_target(key='2',target=[pi/8, 0, 0, 0, 0, -pi/8, 0, 0, 0, 0])


    if key=='0':
        dt = 0.0025
        t = 15
        a = 0
        b = np.pi / 10
        freq_start = 0.5
        freq_end = 1.5
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[i, 0, 0, 0, 0, -i, 0, 0, 0, 0])
            time.sleep(dt)

            

def motor_one_test(key):
    if key=='1':
        change_pos_dof_target(key='1',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='2':
        change_pos_dof_target(key='2',target=[0, pi/8, 0, 0, 0, 0, -pi/8, 0, 0, 0])
    if key=='3':
        change_pos_dof_target(key='3',target=[0, pi/4, 0, 0, 0, 0, -pi/4, 0, 0, 0])
    if key=='4':
        change_pos_dof_target(key='4',target=[0, 3*pi/8, 0, 0, 0, 0, -3*pi/8, 0, 0, 0])
    if key=='0':
        dt = 0.0025
        t = 15
        a = -pi/15
        b = pi/15
        freq_start = 0.2
        freq_end = 1

        x = np.arange(0, t, dt)

        frequencies = np.linspace(freq_start, freq_end, x.size)

        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[0, i, 0, 0, 0, 0, -i, 0, 0, 0])
            time.sleep(dt)

def motor_two_test(key):
   
    if key=='1':
        change_pos_dof_target(key='1',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='2':
        change_pos_dof_target(key='2',target=[0, 0, pi/20, 0, 0, 0, 0, 0, 0, 0])
    if key=='3':
        change_pos_dof_target(key='3',target=[0, 0, pi/10, 0, 0, 0, 0, 0, 0, 0])
    if key=='4':
        change_pos_dof_target(key='4',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='0':
        dt = 0.0025
        t = 15
        a = pi/20
        b = pi/10
        freq_start = 0.2
        freq_end = 1

        x = np.arange(0, t, dt)

        frequencies = np.linspace(freq_start, freq_end, x.size)

        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[0, 0, i, 0, 0, 0, 0, 0, 0, 0])
            time.sleep(dt)


def motor_three_test(key):
    if key=='1':
        change_pos_dof_target(key='1',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='2':
        change_pos_dof_target(key='2',target=[0, 0, 0, pi/8, 0, 0, 0, 0, -pi/8, 0])
    if key=='3':
        change_pos_dof_target(key='3',target=[0, 0, 0, pi/4, 0, 0, 0, 0, -pi/4, 0])
    if key=='4':
        change_pos_dof_target(key='4',target=[0, 0, 0, 3*pi/8, 0, 0, 0, 0, -3*pi/8, 0])
    if key=='0':
        dt = 0.0025
        t = 15
        a = pi/8
        b = pi/4
        freq_start = 0.2
        freq_end = 1

        x = np.arange(0, t, dt)

        frequencies = np.linspace(freq_start, freq_end, x.size)

        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[0, 0, 0, i, 0, 0, 0, 0, -i, 0])
            time.sleep(dt)

def motor_four_test(key):
    if key=='1':
        change_pos_dof_target(key='2',target=[0, 0, 0, 0, -pi/5, 0, 0, 0, 0, pi/5])
    if key=='2':
        change_pos_dof_target(key='3',target=[0, 0, 0, 0, -pi/8, 0, 0, 0, 0, pi/8])
    if key=='3':
        change_pos_dof_target(key='1',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if key=='4':
        change_pos_dof_target(key='3',target=[0, 0, 0, 0, pi/8, 0, 0, 0, 0, -pi/8])
    if key=='5':
        change_pos_dof_target(key='4',target=[0, 0, 0, 0, pi/5, 0, 0, 0, 0, -pi/5])
    if key=='0':
        dt = 0.0025
        t = 15
        a = -pi/12
        b = -pi/5
        freq_start = 0.2
        freq_end = 1

        x = np.arange(0, t, dt)

        frequencies = np.linspace(freq_start, freq_end, x.size)

        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[0, 0, 0, 0, i, 0, 0, 0, 0, -i])
            time.sleep(dt)

def verification(key):
    
    if key=='0':
        a = 0
        b = pi / 10
        dt = 0.0025
        t = 10
        freq_start = 0.3
        freq_end = 1
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='0',target=[i, 0, 0, 0, 0, -i, 0, 0, 0, 0])
            time.sleep(dt)
    if key=='1':
        a = -pi/15
        b = pi/15
        dt = 0.0025
        t = 10
        freq_start = 0.3
        freq_end = 1
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='1',target=[0, i, 0, 0, 0, 0, -i, 0, 0, 0])
            time.sleep(dt)
    if key=='2':
        a = pi/20
        b = pi/10
        dt = 0.0025
        t = 10
        freq_start = 0.3
        freq_end = 1
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='2',target=[0, 0, i, 0, 0, 0, 0, -i, 0, 0])
            time.sleep(dt)
    if key=='3':
        a = pi/8
        b = pi/4
        dt = 0.0025
        t = 10
        freq_start = 0.3
        freq_end = 1
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='3',target=[0, 0, 0, i, 0, 0, 0, 0, -i, 0])
            time.sleep(dt)
    if key=='4':
        a = -pi/12
        b = -pi/5
        dt = 0.0025
        t = 10
        freq_start = 0.3
        freq_end = 1
        x = np.arange(0, t, dt)
        frequencies = np.linspace(freq_start, freq_end, x.size)
        y = (a + b) / 2 + (b - a) / 2 * np.cos(2 * np.pi * frequencies * x + np.pi)
        for i in y:
            change_pos_dof_target(key='4',target=[0, 0, 0, 0, i, 0, 0, 0, 0, -i])
            time.sleep(dt)
    if key == 'r':
        change_pos_dof_target(key='reset',target=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

def walking_test(key):
    import os
    import pandas as pd

    # Read trajectory data from CSV file
    output_folder = '/home/grl/repo/legged_env/envs/scripts/trajectories'
    input_file = os.path.join(output_folder, 'theta_phi_trajectories.csv')
    trajectory_data = pd.read_csv(input_file)
    theta_path = trajectory_data['theta'].values
    phi_path = trajectory_data['phi'].values

    positions = []
    for i in range(len(theta_path)):
        #positions.append([0, theta_path[i], 0, 0, theta_path[i], 0, 0, 0, 0, 0])
        #positions.append([0, 0, 0, -phi_path[i], phi_path[i], 0, 0, 0, 0, 0])
        positions.append([0, theta_path[i], 0, -phi_path[i], theta_path[i] + phi_path[i], 0, 0, 0, 0, 0])
    if key == '0':

        dt = 0.04
    while(True):
        for pos in positions:
            change_pos_dof_target(key='0',target=pos)
            time.sleep(dt)

def press(key):

    # motor_zero_test(key)
    # motor_one_test(key)
    # motor_two_test(key)
    # motor_three_test(key)
    # motor_four_test(key)
    #verification(key)
    walking_test(key)
    if key=='9':
        data = {
            "reset":True,
            "id":random.randint(-sys.maxsize, sys.maxsize)
        }
        # reset
        publish(data)
        print("9 pressed")
        
listen_keyboard(on_press=press,delay_second_char=0.005,delay_other_chars=0.005,sleep=0.0001)


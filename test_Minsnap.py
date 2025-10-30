import minsnap_trajectories as ms
import numpy as np
import numpy.linalg as la
import math
#from pyquaternion import Quaternion
from pyrr import quaternion, Matrix33, Matrix44, Vector3 # array inputs are all flattened
from pynput import keyboard
import time
import matplotlib.pyplot as plt
import torch

if __name__ == '__main__':
    # refs = [
    # ms.Waypoint(
    #     time=0.0,
    #     position=np.array([0.0, 0.0, 10.0]),
    # ),
    # ms.Waypoint(  # Any higher-order derivatives
    #     time=8.0,
    #     position=np.array([10.0, 0.0, 10.0]),
    #     velocity=np.array([0.0, 5.0, 0.0]),
    #     acceleration=np.array([0.1, 0.0, 0.0]),
    # ),
    # ms.Waypoint(  # Potentially leave intermediate-order derivatives unspecified
    #     time=16.0,
    #     position=np.array([20.0, 0.0, 10.0]),
    #     jerk=np.array([0.1, 0.0, 0.2]),
    # ),
    # ]

    # polys = ms.generate_trajectory(
    #     refs,
    #     degree=8,  # Polynomial degree
    #     idx_minimized_orders=(3, 4),  
    #     num_continuous_orders=3,  
    #     algorithm="closed-form",  # Or "constrained"
    # )

    # t = np.linspace(0, 16, 100)
    # #  Sample up to the 3rd order (Jerk) -----v
    # pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6

    # # available order in derivatives in pva is one lesser than computed meaning up to one derivative less  

    # print(pva[5,99,2]) # up to order of derivatives available, waypoints, axis

    

    """ ## Module below used to test the attitude controller
    z = np.array([2,3,1]) # 3 x 1
    z = np.array([[2,3,1]]) # 1 x 3

    z = np.transpose(z)
    print(np.shape(z))

    qz = quaternion.create(0,0,0,1)
    qzi = quaternion.inverse(qz)
    ez = np.array([0,0,1]) # 3,: flattened form

    #a = np.array([[0,0,1]]) # 1 x 3 - row based simulated disk vector 
    a = quaternion.apply_to_vector(qz, ez) # flattened array
    a = np.array([[a[0],a[1],a[2]]])  # 1 x 3 - row based simulated disk vector
    b = np.array([[1,0,1]]) # 1 x 3 - row based simulated zd which is desired vector 
    c = np.dot(a,np.transpose(b)) # 1 x 1
    d = la.norm(a,2)*la.norm(b,2) # L2 norm of a and b
    angle = math.acos(c/d)

    n = np.cross(a,b)/la.norm(np.cross(a,b)) # cross product of a and b - 1 x 3
    #n = np.transpose(n) # 3 x 1
    n = list(n.flat)
    B = quaternion.apply_to_vector(qzi, n) # inverse of qz applied to n
    

    error_quat = np.array([math.cos(angle/2), B[0]*math.sin(angle/2), B[1]*math.sin(angle/2), B[2]*math.sin(angle/2)]) # abt w x y z

    if error_quat[0] < 0:
        bod_rates = -2*error_quat[1:3]
    else:
        bod_rates = 2*error_quat[1:3] # bod_rates[0] = abt x, bod_rates[1] = abt y 
    print(bod_rates) """
    
    # y = 1
    # x = 2
    # heading = math.degrees(math.atan2(x,y)) # its bounded by the 180 deg block
    # print(heading)

    # test_x = np.array([1,2,3])/1
    # print(test_x)


    """ word = 'hello'

    # Setup keyboard listener
    def on_press(key):
        global word
        try:
            if key.char == 'a':  # Check if 'a' is pressed
                word = 'world'
            elif key.char == 'b':  # Check if 'b' is pressed
                word = 'bitch'
            elif key.char == 'c':  # Check if 'b' is pressed
                word = 'hello'
        except AttributeError:
            pass  # Ignore special keys

    # Start listening for key presses
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while True:
        print(word) """
        
    """ radius = 1.3
    t = np.linspace(0, 2*np.pi, num=100)

    x = radius * np.cos(t) / (np.sin(t)**2 + 1)
    y = radius * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1)

    x_dist = radius * 4
    y_dist = (radius/3)*4
 
    laps = 4
    x_set = x
    for i in range(laps):
        x_set = np.concatenate((x_set,x))
    
    print(len(x_set))

    plt.plot(x, y)
    plt.show() """


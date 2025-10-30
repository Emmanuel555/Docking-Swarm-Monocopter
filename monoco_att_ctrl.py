import logging
import time

import Mocap
import DataSave
import Data_process_swarm

import math
from scipy.optimize import fsolve
from pyrr import quaternion
import numpy as np
import numpy.linalg as la


class att_ctrl(object):
    def __init__(self,p_gains,d_gains,i_gains,vel_gains,angle_gains,body_rate_gains,body_rate_rate_gains):
        ## feedback
        self.robot_pos = np.array([0.0,0.0,0.0]) # x y z
        self.robot_vel = np.array([0.0,0.0,0.0]) # x y z
        self.robot_acc = np.array([0.0,0.0,0.0]) # x y z
        self.robot_quat = np.array([0.0,0.0,0.0,0.0]) # x y z w
        self.robot_quat_x = np.array([0.0,0.0,0.0,0.0]) # x y z w
        self.robot_quat_y = np.array([0.0,0.0,0.0,0.0]) # x y z w
        self.robot_tpp_bod_rate = np.array([0.0,0.0,0.0]) # x y z
        self.robot_tpp_bod_raterate = np.array([0.0,0.0,0.0]) # x y z
        self.yaw = 0.0
        self.yawrate = 0.0
        
        ## gains
        # pos
        self.kp = np.array(p_gains) 
        self.kd = np.array(d_gains) 
        self.ki = np.array(i_gains) 
        # vel
        self.kpvel = np.array(vel_gains) 
        #self.kdvel = np.array(vel_gains_d) 
        #self.kivel = np.array(vel_gains_i) 
        # attitude/angle
        # self.kpa = np.array([10, 10]) # abt x y
        self.kpa = np.array(angle_gains) # abt x y
        #self.kpad = np.array(angle_gains_d) # abt x y
        #self.kpai = np.array(angle_gains_i) # abt x y
        # body rates
        # self.kpr = np.array([50, 50]) # abt x y
        self.kpr = np.array(body_rate_gains) # abt x y
        #self.kprd = np.array(body_rate_gains_d) # abt x y
        #self.kpri = np.array(body_rate_gains_i) # abt x y
        # body rate rates
        # self.kprr = np.array([1000, 1000]) # abt x y
        self.kprr = np.array(body_rate_rate_gains) # abt x y # control effectiveness
        #self.kprrd = np.array(body_rate_rate_gains_d) # abt x y
        #self.kprri = np.array(body_rate_rate_gains_i) # abt x y
        ## sampling time
        self.dt = 0

        ## references
        self.ref_pos = np.array([0.0,0.0,0.0]) 
        self.ref_pos_z = 0.0
        self.ref_vel = np.array([0.0,0.0,0.0]) 
        self.ref_acc = np.array([0.0,0.0,0.0]) 
        self.ref_jer = np.array([0.0,0.0,0.0]) 
        self.ref_sna = np.array([0.0,0.0,0.0]) 
        
        ## physical parameters
        self.g = 9.81
        self.rho = 1.225
        self.wing_radius = 0
        self.chord_length = 0
        self.mass = 0
        self.cl = 0
        self.cd = 0
        self.J = np.array([0.0,0.0,0.0]) # inertia matrix

        ## bem
        self.drag_rotation_wo_rps = 0
        self.lift_rotation_wo_rps = 0

        ## control signals
        self.position_error_last = np.array([0.0, 0.0, 0.0])
        self.velocity_error_last = np.array([0.0, 0.0, 0.0])
        self.angle_error_last = np.array([0.0, 0.0])
        self.rate_error_last = np.array([0.0, 0.0])
        self.raterate_error_last = np.array([0.0, 0.0])
        self.p_control_signal = np.array([0.0,0.0,0.0]) 
        self.v_control_signal = np.array([0.0,0.0,0.0]) 
        self.z_offset = 0.0
        self.cmd_z = 0.0
        self.cmd_att = np.array([0.0,0.0])
        self.cmd_bod_rates_final = np.array([0.0,0.0])
        self.cmd_bod_raterates_final = np.array([0.0,0.0])
        self.cascaded_ref_bod_rates = np.array([0.0,0.0])
        self.des_rps = 0.0
        self.z_error = 0.0

        # differential flatness
        self.ref_rates = np.array([0.0,0.0])
        self.ref_raterates = np.array([0.0,0.0])

        # attitude error tracking
        self.attitude_error = 0.0
        self.attitude_rate_error = 0.0
        self.attitude_raterate_error = 0.0


    def physical_params(self,wing_radius,chord_length,mass,cl,cd, J):
        # cl = lift coefficient
        # cd = drag coefficient
        self.wing_radius = wing_radius
        self.chord_length = chord_length
        self.mass = mass
        self.cl = cl
        self.cd = cd
        self.J = J


    def compute_bem_wo_rps(self,pitch):
        # pitch = pitch angle in degrees 
        # original formula for lift: (cl*pitch*(rps^2)*rho*chord_length*(wing_radius^3))/6
        self.drag_rotation_wo_rps = (self.cd*pitch*self.rho*self.chord_length*(self.wing_radius**3))/6 # has mass inside, not needed atm
        # print ("pitch: ", pitch)
        if pitch <= 0.0:
            pitch = 0.0
        
        self.lift_rotation_wo_rps = (self.cl*pitch*self.rho*self.chord_length*(self.wing_radius**3))/6 # has mass inside


    def linear_ref(self,ref_pos,ref_vel,ref_acc,ref_jer,ref_sna,ref_pos_z):
        self.ref_pos = ref_pos
        self.ref_pos_z = ref_pos_z
        self.ref_vel = ref_vel
        self.ref_acc = ref_acc    
        self.ref_jer = ref_jer
        self.ref_sna = ref_sna 


    def update(self, linear_pos, rotational_pos, rotational_quat, dt, z_offset, yaw, quat_x, quat_y, yawrate):
        self.z_offset = z_offset
        self.robot_pos = np.array(linear_pos[0:3])
        self.robot_vel = np.array(linear_pos[3:6])
        self.robot_acc = np.array(linear_pos[6:9])
        self.robot_quat = rotational_quat
        self.robot_tpp_bod_rate = np.array(rotational_pos[1])
        self.robot_tpp_bod_raterate = np.array(rotational_pos[2])
        self.dt = dt
        self.yaw = yaw
        self.robot_quat_x = quat_x
        self.robot_quat_y = quat_y
        self.yawrate = yawrate
        

    def update_ref_pos(self, ref_pos):
        self.ref_pos = ref_pos 


    def attitude_loop(self, quat, control_input):
        kpa = self.kpa # abt x y
        #kpad = self.kpad # abt x y
        #kpai = self.kpai # abt x y
        qz = quaternion.create(quat[0], quat[1], quat[2], 1.0) # x y z w ## qw is always set to 1 even in optitrack itself
        #qz = np.array([quat[0],quat[1],quat[2],quat[3]]) # x y z w
        qzi = quaternion.inverse(qz)
        ez = np.array([0, 0, 1]) # 3,:
        disk_vector = quaternion.apply_to_vector(qz, ez) # flattened array
        disk_vector = np.array([[disk_vector[0],disk_vector[1],disk_vector[2]]])  # 1 x 3 - row based simulated disk vector
        
        ## pos control input here
        # if la.norm(control_input,2) == 0:
        #     zd = np.array([0.0,0.0,0.0])
        # else:
        #     zd = control_input/la.norm(control_input,2) # unit vector

        zd_n = control_input   
        zd_n = np.array([[zd_n[0],zd_n[1],zd_n[2]]]) # 1 x 3 - row based simulated zd which is desired vector 
        
        #print ("zd_n: ", zd_n)
        #zd_d = zd_n/la.norm(zd_n,2)
        #print ("disk vector: ", disk_vector)
        #print ("control_input: ", control_input)

        ## prev for 65
        #zd_d = np.array([[zd_d[0],zd_d[1],zd_d[2]]])
        #zd_d = np.multiply(zd_d,zd_n)
        
        num = np.dot(disk_vector,np.transpose(zd_n)) # 1 x 1
        den = la.norm(disk_vector,2)*la.norm(zd_n,2) # L2 norm of a and b
        num_den = num/den 
        #print ("num/den: ", num/den)

        if abs(num_den) > 1.0:
            num_den = 1.0*(num_den/abs(num_den))
        
        angle = math.acos(num_den) # angle in radians
        
        # print("disk_vector: ", disk_vector)
        # print("zd_n: ", zd_n)
        # print("n_dem: ", la.norm(np.cross(disk_vector,zd_n)))


        if la.norm(np.cross(disk_vector,zd_n)) == 0.0:
            n = 0.0
            cmd_att = np.array([0.0,0.0])
        else:
            n = np.cross(disk_vector,zd_n)/la.norm(np.cross(disk_vector,zd_n)) # cross product of a and b - 1 x 3
            n = list(n.flat) # flattened array
            B = quaternion.apply_to_vector(qzi, n) # inverse of qz applied to n
            error_quat = np.array([math.cos(angle/2), B[0]*math.sin(angle/2), B[1]*math.sin(angle/2), B[2]*math.sin(angle/2)]) # abt w x y z

            if error_quat[0] < 0:
                cmd_att = -2*error_quat[1:3]
            else:
                cmd_att = 2*error_quat[1:3] # bod_att[0] = abt x, bod_att[1] = abt y    

        #cmd_att_error = np.array([cmd_att[0],cmd_att[1]]) # abt x y only

        cmd_att_error = np.array([cmd_att[1],cmd_att[0]]) # abt y x only
        self.attitude_error = cmd_att_error 
        cmd_att_final = kpa*(cmd_att_error)
        return (cmd_att_final)
    

    def p_control_input_manual(self,manual_input):
        self.p_control_signal = manual_input
    

    def p_control_input(self,sampling_dt):
        # control gains
        p_gains = self.kp # p_gains = ([kpx, kpy, kpz])
        d_gains = self.kd # d_gains = ([kpx, kpy, kpz])
        i_gains = self.ki # i_gains = ([kpx, kpy, kpz])
        
        #I_term_z_prior = 0.0
        #I_term_prior = np.array([0.0, 0.0, I_term_z_prior])

        position_error = self.ref_pos - self.robot_pos # calculate position error
        rate_posiition_error = (position_error - self.position_error_last)/sampling_dt
        #integral_error = (position_error*sampling_dt) + I_term_prior
        #self.position_error_last = position_error
        
        # position pid controller
        #self.p_control_signal = (p_gains * position_error) + (d_gains * rate_posiition_error) + (i_gains * integral_error) 
        #self.p_control_signal = np.multiply(p_gains,position_error) 
        self.p_control_signal = (p_gains * position_error) + (d_gains * rate_posiition_error)

        # position control loop saturation
        if abs(self.p_control_signal[0]) > 1.0:
            self.p_control_signal[0] = 1.0*(self.p_control_signal[0]/abs(self.p_control_signal[0]))
        if abs(self.p_control_signal[1]) > 1.0:
            self.p_control_signal[1] = 1.0*(self.p_control_signal[1]/abs(self.p_control_signal[1]))    
        self.p_control_signal[2] = 1.0

        #self.p_control_signal = np.array([0.0,0.0,1.0]) # abt x y z
        #self.p_control_signal = manual_input


    def v_control_input(self):
        #vel_gains_p = self.kpvel

        # add in vel controller
        #v_ref = self.ref_vel
        #v_error = v_ref - self.robot_vel
        #v_rate_error = (v_error - self.velocity_error_last)/sampling_dt
        #v_integral_error = (v_error*sampling_dt)
        #self.velocity_error_last = v_error
        #self.v_control_signal = vel_gains_p * v_error

        # ref acceleration
        ref_acc = np.array([self.ref_acc[0],self.ref_acc[1],0.0])
        
        # w acceleration references
        # self.v_control_signal = self.v_control_signal + ref_acc # abt x y z
        self.v_control_signal = ref_acc # abt x y z


    def collective_thrust(self,kpz,kdz,kiz): 
        # weight of the robot
        robot_mg = np.array([0.0,0.0,self.mass*self.g]) # robot weight, cf = 47500
        self.z_error = self.ref_pos_z - self.robot_pos[2]
        rate_position_error_z = (self.z_error - self.position_error_last[2])/self.dt
        integral_error_z = (self.z_error*self.dt)
        self.position_error_last[2] = self.z_error 

        p_error_z = kpz*(self.z_error) + kdz*(rate_position_error_z) + robot_mg[2] + kiz*(integral_error_z) + self.ref_acc[2] # z error
        
        self.des_rps = p_error_z
        if self.lift_rotation_wo_rps == 0.0:
            des_thrust = 0.0
        else:
            des_thrust = self.lift_rotation_wo_rps*(self.yawrate**2)
        self.cmd_z = des_thrust

        if self.des_rps > 55500:
            self.des_rps = 55500

        return (self.des_rps,self.cmd_z)
    

    def manual_collective_thrust(self,kpz,kdz,kiz,manual_thrust): 
        # weight of the robot
        robot_mg = np.array([0.0,0.0,self.mass*self.g]) # robot weight, cf = 47500
        self.z_error = self.ref_pos_z - self.robot_pos[2]
        rate_position_error_z = (self.z_error - self.position_error_last[2])/self.dt
        integral_error_z = (self.z_error*self.dt)
        self.position_error_last[2] = self.z_error 

        p_error_z = kpz*(self.z_error) + kdz*(rate_position_error_z) + robot_mg[2] + kiz*(integral_error_z) + self.ref_acc[2] # z error
        
        self.des_rps = p_error_z + manual_thrust
        if self.lift_rotation_wo_rps == 0.0:
            des_thrust = 0.0
        else:
            des_thrust = self.lift_rotation_wo_rps*(self.yawrate**2)
        self.cmd_z = des_thrust

        return (self.des_rps,self.cmd_z)
        

    def body_rate_loop(self,cascaded_ref_bod_rates):
        kpr = self.kpr
        fb = np.array(self.robot_tpp_bod_rate[0:2]) # abt x y z

        # body rate controller
        cmd_bod_rates_error = cascaded_ref_bod_rates - fb
        self.attitude_rate_error = cmd_bod_rates_error
        self.cmd_bod_rates_final = kpr*(cmd_bod_rates_error)
        return (self.cmd_bod_rates_final)
    

    def INDI_loop(self,cascaded_ref_bod_acc):
       
        kprr = self.kprr
       
        fb = np.array(self.robot_tpp_bod_raterate[0:2]) # abt x y z
        cmd_bod_acc_error = cascaded_ref_bod_acc - fb
        self.attitude_raterate_error = cmd_bod_acc_error
        cmd_bod_acc_final = kprr*(cmd_bod_acc_error) 
        self.cmd_bod_raterates_final = cmd_bod_acc_final # for logging purposes

        ## NDI
        #cmd_bod_acc_final = kprr*(cascaded_ref_bod_acc)
        return (cmd_bod_acc_final)
    
    
    def get_angle(self): # corrected
        control_input = self.p_control_signal #+ self.v_control_signal
        cmd_att = self.attitude_loop(self.robot_quat, control_input)
        self.cmd_att = cmd_att #rpy
        return (self.cmd_att)
    

    def get_body_rate(self,flatness_option):
        if flatness_option == 0:
            self.cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
        else:
            self.cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att + self.include_jerk_bod_rates())
        return (self.cascaded_ref_bod_rates)
    

    def CF_SAM_get_angles_and_thrust(self,enable,flatness_option):
        # output saturation/normalisation
        des_rps = self.des_rps
        #cmd_bod_acc = self.cmd_att
        
        ## icra method
        #cmd_bod_acc = self.cmd_att + self.include_snap_bod_raterate() + self.include_jerk_bod_rates()


        if flatness_option == 0:
            #cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
        else:
            #cascaded_ref_bod_rates = self.body_rate_loop(cmd_att)
            #cascaded_ref_bod_rates = self.include_jerk_bod_rates() + cascaded_ref_bod_rates
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
            cmd_bod_acc = self.include_snap_bod_raterate() + cmd_bod_acc
            #cmd_att = cmd_att + self.include_snap_bod_raterate() + self.include_jerk_bod_rates()
        

        ## to account for phase delay
        x_sign = math.sin(self.yaw)
        y_sign = math.cos(self.yaw)

        cmd_bod_acc[0] = cmd_bod_acc[0] * y_sign * -1 
        cmd_bod_acc[1] = cmd_bod_acc[1] * x_sign * -1
        

        # output saturation (cmd_att)
        if abs(cmd_bod_acc[0]) > 10000:
            cmd_bod_acc[0] = 10000*(cmd_bod_acc[0]/abs(cmd_bod_acc[0]))        
        if abs(cmd_bod_acc[1]) > 10000:
             cmd_bod_acc[1] = 10000*(cmd_bod_acc[1]/abs(cmd_bod_acc[1]))


        # motor saturation
        # if final_motor_output > 65500:
        #     final_motor_output = 65500
        # elif final_motor_output < 10:
        #     final_motor_output = 10

        #final_cmd = np.array([final_motor_output, final_motor_output, final_motor_output, final_motor_output])
        
        #self.cmd_z = enable*des_thrust

        #return (final_cmd)
        return (cmd_bod_acc)

    
    """ def get_angles_and_thrust(self,flatness_option,amplitude):
        # self.control_input()
        # cmd_att = self.attitude_loop(self.robot_quat, self.control_signal)


        if flatness_option == 0:
            #cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
        else:
            #cascaded_ref_bod_rates = self.body_rate_loop(cmd_att)
            #cascaded_ref_bod_rates = self.include_jerk_bod_rates() + cascaded_ref_bod_rates
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
            cmd_bod_acc = self.include_snap_bod_raterate(amplitude) + cmd_bod_acc
            #cmd_att = cmd_att + self.include_snap_bod_raterate() + self.include_jerk_bod_rates()
        
        
        # testing INDI
        #des_roll_rate = float(self.cmd_att[0]/ref_sampling_dt)
        #des_pitch_rate = float(self.cmd_att[1]/ref_sampling_dt)

        #des_roll_raterate = float(des_roll_rate/ref_sampling_dt)
        #des_pitch_raterate = float(des_pitch_rate/ref_sampling_dt)

        #cascaded_ref_bod_rates = np.array([des_roll_raterate, des_pitch_raterate])
        #cmd_bod_acc = self.INDI_loop(cascaded_ref_bod_rates)

        #cmd_bod_acc = self.cmd_att
        #cmd_bod_acc = self.cascaded_ref_bod_rates

        ## resultant precession control
        # cmd_bod_acc = (cmd_bod_acc*self.J[0])/(self.J[2]*self.yawrate)
        final_des_roll_raterate = float(cmd_bod_acc[0])
        final_des_pitch_raterate = float(cmd_bod_acc[1])


        # output saturation/normalisation
        des_rps = self.des_rps/30
        

        if abs(des_rps) > 1.0:
            des_rps = 1.0*(des_rps/abs(des_rps))


        ## precession sign change
        #x_sign = abs(math.sin(self.yaw))
        #y_sign = abs(math.cos(self.yaw))


        ## when involving pitch roll - 1/100000 = moment of inertia
        des_x = (final_des_pitch_raterate/(self.wing_radius*self.mass))/100000 # convert to linear term cos of inner cyclic ctrl
        des_y = (-1*final_des_roll_raterate/(self.wing_radius*self.mass))/100000


        ## compare against pid control
        des_x = self.p_control_signal[0]/50
        des_y = self.p_control_signal[1]/50
        
        
        if abs(des_x) > 1.0:
            des_x = 1.0*(des_x/abs(des_x))

        if abs(des_y) > 1.0:
            des_y = 1.0*(des_y/abs(des_y))
        

        cyclic_gain = 1000000 # 800000
        collective_gain = 1000000

        ## final cmd at the end
        final_cmd = np.array([[des_x*cyclic_gain, des_y*cyclic_gain, des_rps*collective_gain, float(0)]]) # linear(x)-pitch(y), linear(y)-roll(x), rps on wj side
        
        return (final_cmd) """
    

    """ def NDI_get_angles_and_thrust(self,flatness_option):
        # self.control_input()
        # cmd_att = self.attitude_loop(self.robot_quat, self.control_signal)

        if flatness_option == 0:
            #cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
            cmd_bod_acc = self.kprr*(self.cascaded_ref_bod_rates)
        else:
            cmd_bod_acc = self.kprr*(self.cascaded_ref_bod_rates)
            cmd_bod_acc = self.include_snap_bod_raterate() + cmd_bod_acc
           
        #cmd_bod_acc = cmd_bod_acc/self.dt
        # angles
        #des_roll = float(self.cmd_att[0])
        #des_pitch = float(self.cmd_att[1])


        # testing INDI
        #des_roll_rate = float(self.cmd_att[0]/ref_sampling_dt)
        #des_pitch_rate = float(self.cmd_att[1]/ref_sampling_dt)

        #des_roll_raterate = float(des_roll_rate/ref_sampling_dt)
        #des_pitch_raterate = float(des_pitch_rate/ref_sampling_dt)

        #cascaded_ref_bod_rates = np.array([des_roll_raterate, des_pitch_raterate])
        #cmd_bod_acc = self.INDI_loop(cascaded_ref_bod_rates)

        
        final_des_roll_raterate = float(cmd_bod_acc[0])
        final_des_pitch_raterate = float(cmd_bod_acc[1])


        # output saturation/normalisation
        des_rps = self.des_rps/1000
        

        if abs(des_rps) > 1.0:
            des_rps = 1.0*(des_rps/abs(des_rps))


        ## when involving pitch roll
        des_x = (final_des_pitch_raterate/(self.wing_radius*self.mass))/10000 # convert to linear term cos of inner cyclic ctrl
        des_y = (-1*final_des_roll_raterate/(self.wing_radius*self.mass))/10000


        ## compare against pid control
        #des_x = self.control_signal[0]/100
        #des_y = self.control_signal[1]/100
        
        
        if abs(des_x) > 1.0:
            des_x = 1.0*(des_x/abs(des_x))

        if abs(des_y) > 1.0:
            des_y = 1.0*(des_y/abs(des_y))
        

        cyclic_gain = 1000000
        collective_gain = 1000000

        ## final cmd at the end
        final_cmd = np.array([[des_x*cyclic_gain, des_y*cyclic_gain, des_rps*collective_gain, float(0)]]) # linear(x)-pitch(y), linear(y)-roll(x), rps on wj side
        
        return (final_cmd) """
    

    def precession_rate(self):
        if self.yawrate == 0:
            precession_rate = 0
        else:
            precession_rate = (self.J[0]*la.norm(self.robot_tpp_bod_raterate,2))/(self.J[2]*self.yawrate)
        return (precession_rate,self.yawrate)


    def include_jerk_bod_rates(self):
        wy = self.ref_jer[0]/self.g
        wx = self.ref_jer[1]/(-1*self.g)
        
        ref_bod_rates = np.array([wy,wx]) # flattened array abt x y z
        self.ref_rates = ref_bod_rates 
        
        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_rates
    

    def include_snap_bod_raterate(self):
        wy_dot = self.ref_sna[0]/self.g
        wx_dot = self.ref_sna[1]/(-1*self.g)
        
        ref_bod_raterate = np.array([wy_dot,wx_dot]) # flattened array abt x y z
        self.ref_raterates = ref_bod_raterate

        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_raterate
    
    
"""     def INDI_loop(self):
        # INDI body rate rates
        self.kpang = np.array([1.0, 1.0, 1.0]) # 3,: flattened form
        self.kdang = np.array([0, 0, 0]) # 3,: flattened form
        self.kiang = np.array([0, 0, 0]) # 3,: flattened form

        # commanded inputs
        self.cmd_att = np.array([0, 0, 0]) # attitude
        self.cmd_bod_rates = np.array([0, 0, 0]) # body rates
        self.ff_snap = np.array([0, 0, 0]) # ff snap
        self.cmd_bod_rates_rates = np.array([0, 0, 0]) # body rate rates

        
        if self.mode == 1:
           self.cmd_bod_rates_rates = self.kpang*(self.cmd_att - self.last_angular_rate_rate)
        elif self.mode == 3:
            self.cmd_bod_rates_rates = self.kpang*(((self.cmd_att + self.cmd_bod_rates + self.ff_snap)/3) - self.last_angular_rate_rate) 
        self.cmd_bod_rates_rates = self.MOI*self.cmd_bod_rates_rates
        return self.cmd_bod_rates_rates """



# from Library.IIR2Filter import IIR2Filter
import struct
from Filter import IIR2Filter
import math
import numpy as np


class RealTimeProcessor(object):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate):
        self.flag = 0
        self.sample_time = 1 / sample_rate
        self.sample_rate = sample_rate

        # raw data from motion capture
        self.px1 = 0
        self.py1 = 0
        self.pz1 = 0
        self.quat_x1 = 0
        self.quat_y1 = 0
        self.quat_w1 = 0
        self.quat_z1 = 0

        self.px2 = 0
        self.py2 = 0
        self.pz2 = 0
        self.quat_x2 = 0
        self.quat_y2 = 0
        self.quat_w2 = 0
        self.quat_z2 = 0

        self.px3 = 0
        self.py3 = 0
        self.pz3 = 0
        self.quat_x3 = 0
        self.quat_y3 = 0
        self.quat_w3 = 0
        self.quat_z3 = 0

        # filted data with IIR2Filter
        self.px1_filted = 0
        self.py1_filted = 0
        self.pz1_filted = 0
        self.quat_x1_filted = 0
        self.quat_y1_filted = 0
        self.quat_z1_filted = 0
        self.quat_w1_filted = 0

        self.px2_filted = 0
        self.py2_filted = 0
        self.pz2_filted = 0
        self.quat_x2_filted = 0
        self.quat_y2_filted = 0
        self.quat_z2_filted = 0
        self.quat_w2_filted = 0

        # component of rotation matrix
        self.R11_1 = 0
        self.R12_1 = 0
        self.R13_1 = 0
        self.R21_1 = 0
        self.R22_1 = 0
        self.R23_1 = 0
        self.R31_1 = 0
        self.R32_1 = 0
        self.R33_1 = 0

        self.R11_2 = 0
        self.R12_2 = 0
        self.R13_2 = 0
        self.R21_2 = 0
        self.R22_2 = 0
        self.R23_2 = 0
        self.R31_2 = 0
        self.R32_2 = 0
        self.R33_2 = 0

        # filter setup
        self.FilterX1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQX1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQY1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQZ1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQW1 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

        self.FilterX2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQX2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQY2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQZ2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQW2 = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

    def data_unpack(self, udp_data): # for 3 drones only
        x1, y1, z1, qx1, qy1, qz1, qw1, x2, y2, z2, qx2, qy2, qz2, qw2, x3, y3, z3, qx3, qy3, qz3, qw3= struct.unpack("hhhhhhhhhhhhhhhhhhhhh", udp_data)
        self.px1 = x1 * 0.0005  # position px
        self.py1 = y1 * 0.0005  # position py
        self.pz1 = z1 * 0.0005  # position pz
        self.px2 = x2 * 0.0005  # position px
        self.py2 = y2 * 0.0005  # position py
        self.pz2 = z2 * 0.0005  # position pz
        self.px3 = x3 * 0.0005  # position px
        self.py3 = y3 * 0.0005  # position py
        self.pz3 = z3 * 0.0005  # position pz

        self.quat_x1 = float(qx1 * 0.001)
        self.quat_y1 = float(qy1 * 0.001)
        self.quat_z1 = float(qz1 * 0.001)
        self.quat_w1 = float(qw1 * 0.001)
        self.quat_x2 = float(qx2 * 0.001)
        self.quat_y2 = float(qy2 * 0.001)
        self.quat_z2 = float(qz2 * 0.001)
        self.quat_w2 = float(qw2 * 0.001)
        self.quat_x3 = float(qx3 * 0.001)
        self.quat_y3 = float(qy3 * 0.001)
        self.quat_z3 = float(qz3 * 0.001)
        self.quat_w3 = float(qw3 * 0.001)

        self.raw_data = [self.px1, self.py1, self.pz1, self.quat_x1, self.quat_y1, self.quat_z1, self.quat_w1,
                         self.px2, self.py2, self.pz2, self.quat_x2, self.quat_y2, self.quat_z2, self.quat_w2,
                         self.px3, self.py3, self.pz3, self.quat_x3, self.quat_y3, self.quat_z3, self.quat_w3]

    def get_data_filted(self):

        self.px1_filted = self.FilterX1.filter(self.px1)
        self.py1_filted = self.FilterY1.filter(self.py1)
        self.pz1_filted = self.FilterZ1.filter(self.pz1)
        self.px2_filted = self.FilterX2.filter(self.px2)
        self.py2_filted = self.FilterY2.filter(self.py2)
        self.pz2_filted = self.FilterZ2.filter(self.pz2)

        self.quat_x1_filted = self.FilterQX1.filter(self.quat_x1)
        self.quat_y1_filted = self.FilterQY1.filter(self.quat_y1)
        self.quat_z1_filted = self.FilterQZ1.filter(self.quat_z1)
        self.quat_w1_filted = self.FilterQW1.filter(self.quat_w1)
        self.quat_x2_filted = self.FilterQX2.filter(self.quat_x2)
        self.quat_y2_filted = self.FilterQY2.filter(self.quat_y2)
        self.quat_z2_filted = self.FilterQZ2.filter(self.quat_z2)
        self.quat_w2_filted = self.FilterQW2.filter(self.quat_w2)

        # filted_data = [self.px1_filted, self.py1_filted, self.pz1_filted, self.quat_x1_filted, self.quat_y1_filted,
        #                self.quat_z1_filted, self.quat_w1_filted, self.px2_filted, self.py2_filted, self.pz2_filted,
        #                self.quat_x2_filted, self.quat_y2_filted, self.quat_z2_filted, self.quat_w2_filted]

        filted_data = [self.px1_filted, self.py1_filted, self.pz1_filted, self.quat_x1_filted, self.quat_y1_filted,
                       self.quat_z1_filted, self.quat_w1_filted]

        return filted_data

    def get_rotm_1(self):

        xx1 = self.quat_x1 * self.quat_x1
        yy1 = self.quat_y1 * self.quat_y1
        zz1 = self.quat_z1 * self.quat_z1
        xy1 = self.quat_x1 * self.quat_y1
        xz1 = self.quat_x1 * self.quat_z1
        yz1 = self.quat_y1 * self.quat_z1
        wx1 = self.quat_w1 * self.quat_x1
        wy1 = self.quat_w1 * self.quat_y1
        wz1 = self.quat_w1 * self.quat_z1

        self.R11_1 = 1 - 2 * (yy1 + zz1)
        self.R12_1 = 2 * (xy1 - wz1)
        self.R13_1 = 2 * (xz1 + wy1)
        self.R21_1 = 2 * (xy1 + wz1)
        self.R22_1 = 1 - 2 * (xx1 + zz1)
        self.R23_1 = 2 * (yz1 - wx1)
        self.R31_1 = 2 * (xz1 - wy1)
        self.R32_1 = 2 * (yz1 + wx1)
        self.R33_1 = 1 - 2 * (xx1 + yy1)

        rotm_1 = [self.R11_1, self.R12_1, self.R13_1, self.R21_1, self.R22_1, self.R23_1, self.R31_1, self.R32_1, self.R33_1]
        return rotm_1

    def get_rotm_2(self):

        xx2 = self.quat_x2 * self.quat_x2
        yy2 = self.quat_y2 * self.quat_y2
        zz2 = self.quat_z2 * self.quat_z2
        xy2 = self.quat_x2 * self.quat_y2
        xz2 = self.quat_x2 * self.quat_z2
        yz2 = self.quat_y2 * self.quat_z2
        wx2 = self.quat_w2 * self.quat_x2
        wy2 = self.quat_w2 * self.quat_y2
        wz2 = self.quat_w2 * self.quat_z2

        self.R11_2 = 1 - 2 * (yy2 + zz2)
        self.R12_2 = 2 * (xy2 - wz2)
        self.R13_2 = 2 * (xz2 + wy2)
        self.R21_2 = 2 * (xy2 + wz2)
        self.R22_2 = 1 - 2 * (xx2 + zz2)
        self.R23_2 = 2 * (yz2 - wx2)
        self.R31_2 = 2 * (xz2 - wy2)
        self.R32_2 = 2 * (yz2 + wx2)
        self.R33_2 = 1 - 2 * (xx2 + yy2)

        rotm_2 = [self.R11_2, self.R12_2, self.R13_2, self.R21_2, self.R22_2, self.R23_2, self.R31_2, self.R32_2, self.R33_2]
        return rotm_2


    def get_rotm_3(self):

        xx3 = self.quat_x3 * self.quat_x3
        yy3 = self.quat_y3 * self.quat_y3
        zz3 = self.quat_z3 * self.quat_z3
        xy3 = self.quat_x3 * self.quat_y3
        xz3 = self.quat_x3 * self.quat_z3
        yz3 = self.quat_y3 * self.quat_z3
        wx3 = self.quat_w3 * self.quat_x3
        wy3 = self.quat_w3 * self.quat_y3
        wz3 = self.quat_w3 * self.quat_z3

        self.R11_3 = 1 - 2 * (yy3 + zz3)
        self.R12_3 = 2 * (xy3 - wz3)
        self.R13_3 = 2 * (xz3 + wy3)
        self.R21_3 = 2 * (xy3 + wz3)
        self.R22_3 = 1 - 2 * (xx3 + zz3)
        self.R23_3 = 2 * (yz3 - wx3)
        self.R31_3 = 2 * (xz3 - wy3)
        self.R32_3 = 2 * (yz3 + wx3)
        self.R33_3 = 1 - 2 * (xx3 + yy3)

        rotm_3 = [self.R11_3, self.R12_3, self.R13_3, self.R21_3, self.R22_3, self.R23_3, self.R31_3, self.R32_3, self.R33_3]
        return rotm_3

    def get_heading_x1(self):
        heading_1 = math.atan2(self.R21_1, self.R11_1)
        return heading_1

    def get_heading_x2(self):
        heading_2 = math.atan2(self.R21_2, self.R11_2)
        return heading_2

    def get_heading_x3(self):
        heading_3 = math.atan2(self.R21_3, self.R11_3)
        return heading_3

    # def get_bicop_position(self):
    #
    #     px_robot_1 = self.px1_filted
    #     py_robot_1 = self.py1_filted
    #     pz_robot_1 = self.pz1_filted
    #
    #     px_robot_2 = self.px2_filted
    #     py_robot_2 = self.py2_filted
    #     pz_robot_2 = self.pz2_filted
    #
    #     px_bicop = (px_robot_1 + px_robot_2) / 2
    #     py_bicop = (py_robot_1 + py_robot_2) / 2
    #     pz_bicop = (pz_robot_1 + pz_robot_2) / 2
    #
    #     p_bicop = [px_bicop, py_bicop, pz_bicop]
    #
    #     return p_bicop










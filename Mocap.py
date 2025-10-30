import socket
import time
import numpy as np


class Udp(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222, num_bodies=1): # need to reconfigure here for both sending ips and receiving ips
        self.udpStop = False
        self.udp_data = None
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies=num_bodies
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP receive
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP send
        self.sock_rx.bind((self.udp_ip, self.udp_port))
        self.sample_rate_flag = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate

    def get_sample_rate(self):
        if self.sample_rate_flag == -1:
            print('computing sample rate..')
            time_list = []
            for i in range(100): # get 1000 sample
                time_list.append(time.time())
                data, addr = self.sock_rx.recvfrom(14)  # ubuntu set the buffer size to 14 for one vehicle, 42 for 3 vehicles
            dtime = np.diff(time_list)
            sample_time = np.mean(dtime)
            print('Sample rate: %.2f' % (1/sample_time), 'Hz')
            return 1/sample_time
        else:
            return self.sample_rate

    # def udp_step(self):
    #     udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
    #     return udp_data

    def get_data(self):
        #udp_data, addr = self.sock_rx.recvfrom(42)
        #udp_data, addr = self.sock_rx.recvfrom(28)
        udp_data, addr = self.sock_rx.recvfrom(14) # ubuntu is very exact with this number, 7*2, 7*2*3
        return udp_data
    
    def send_data(self, UDP_IP, UDP_PORT, data): # data is a byte array
        #UDP_IP = "127.0.0.1"
        #UDP_PORT = 5005
         
        #print("UDP target IP: %s" % UDP_IP)
        #print("UDP target port: %s" % UDP_PORT)
        #print("Data sent is: %s" % data)
         
        #esp32_sock = socket.socket(socket.AF_INET, # Internet
        #                      socket.SOCK_DGRAM) # UDP
        
        #esp32_sock.sendto(data, (UDP_IP, UDP_PORT))
        
        self.sock_tx.sendto(data, (UDP_IP, UDP_PORT))

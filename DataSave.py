import time
import scipy.io as sio


class SaveData(object):

    def __init__(self, *args):
        self.item_list = args
        self.item_size = len(self.item_list)
        for i in range(self.item_size):    # create empty list as the same size as  item size
            if i == 0:
                self.item = ([],)
            else:
                self.item = self.item + ([],)

    def add_item(self, *args):
        for i in range(self.item_size):
            self.item[i].append(args[i])

    def save_data(self, path):
        time_stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
        file_name = path + time_stamp + '.mat'
        data_dict = {'exptime':time_stamp, }
        for i in range(self.item_size):
            data_dict.update({self.item_list[i]: self.item[i]})

        sio.savemat(file_name, data_dict)
        print('Data saved: ' + file_name)

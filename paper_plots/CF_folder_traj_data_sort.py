import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import CF_all_traj_data_compile as cf


class sort_traj_data(object):
    def __init__(self):
        self.traj_compile = cf.all_trajectory()

    
    def plot_circle(self,folder,att): # only for indi
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
       
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        
        if att == 1:
            indi3 = self.traj_compile.att_data_compiler(indi3)
            ndi3 = self.traj_compile.att_data_compiler(ndi3)
            
            indi5 = self.traj_compile.att_data_compiler(indi5)
            ndi5 = self.traj_compile.att_data_compiler(ndi5)
        else:
            indi3 = self.traj_compile.traj_data_compiler(indi3)
            ndi3 = self.traj_compile.traj_data_compiler(ndi3)
            
            indi5 = self.traj_compile.traj_data_compiler(indi5)
            ndi5 = self.traj_compile.traj_data_compiler(ndi5)
        
    
        return (indi3, ndi3, indi5, ndi5)
    

    def whisker_circle(self,folder):
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
        att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/att/'

        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/att/'

        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)
        att3 = self.traj_compile.data_whisker_plot(att3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        att5 = self.traj_compile.data_whisker_plot(att5)
        
        return (indi3, ndi3, att3, indi5, ndi5, att5)
    

    def foam_whisker_circle(self,folder):
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
        
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        

        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        
        return (indi3, ndi3, indi5, ndi5)


    
    def plot_elevated_circle(self,folder,att):
        # elevated circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/ndi/'
        
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/ndi/'
     
        if att == 1:
            indi3 = self.traj_compile.att_data_compiler(indi3)
            ndi3 = self.traj_compile.att_data_compiler(ndi3)
            
            indi5 = self.traj_compile.att_data_compiler(indi5)
            ndi5 = self.traj_compile.att_data_compiler(ndi5)
        else:
            indi3 = self.traj_compile.traj_data_compiler(indi3)
            ndi3 = self.traj_compile.traj_data_compiler(ndi3)
            
            indi5 = self.traj_compile.traj_data_compiler(indi5)
            ndi5 = self.traj_compile.traj_data_compiler(ndi5)
   
        return (indi3, ndi3, indi5, ndi5)


    def whisker_elevated_circle(self,folder):
        # elevated circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/ndi/'
        att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/att/'

        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/ndi/'
        att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/att/'


        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)
        att3 = self.traj_compile.data_whisker_plot(att3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        att5 = self.traj_compile.data_whisker_plot(att5)

        return (indi3, ndi3, att3, indi5, ndi5, att5)
    

    def foam_whisker_elevated_circle(self,folder):
        # elevated circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/ndi/'
        
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/ndi/'
        

        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)
       
        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
       
        return (indi3, ndi3, indi5, ndi5)
    

    def plot_lem(self,folder,att):
        # lem 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/ndi/' 
        
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/ndi/'
        
        if att == 1:
            indi3 = self.traj_compile.att_data_compiler(indi3)
            ndi3 = self.traj_compile.att_data_compiler(ndi3)
            
            indi5 = self.traj_compile.att_data_compiler(indi5)
            ndi5 = self.traj_compile.att_data_compiler(ndi5)
        else:
            indi3 = self.traj_compile.traj_data_compiler(indi3)
            ndi3 = self.traj_compile.traj_data_compiler(ndi3)
            
            indi5 = self.traj_compile.traj_data_compiler(indi5)
            ndi5 = self.traj_compile.traj_data_compiler(ndi5)

        return (indi3, ndi3, indi5, ndi5)
    

    def whisker_lem(self,folder):
        # lem 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/ndi/' 
        att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/att/'

        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/ndi/'
        att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/att/'


        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)
        att3 = self.traj_compile.data_whisker_plot(att3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        att5 = self.traj_compile.data_whisker_plot(att5)

        return (indi3, ndi3, att3, indi5, ndi5, att5)
    
    
    
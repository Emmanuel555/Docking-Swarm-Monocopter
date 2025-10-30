import numpy as np
import timeit


class drag_estimator(object):
    def __init__(self, step_size, init_values, array_size, order_duration, sampling_time):
        self.step_size = step_size # array
        self.drag_terms = np.zeros((order_duration/sampling_time,array_size))
        self.rmse = np.zeros((order_duration/sampling_time))
        self.drag_rmse_terms = [[self.drag_terms,self.rmse]]
        self.order_ready = False
        self.init_values = init_values

        for i in range(len(self.drag_rmse_terms)):
            self.drag_rmse_terms[i][0] = self.init_values + self.step_size*i
            
        # phases
        self.p1 = True
        self.p2 = False
        self.p3 = False
        self.p4 = False
        self.p5 = False
        self.p61 = False # contraction phase 1
        self.p62 = False # contraction phase 2

        # NM parameters
        self.count = 0
        self.start_accumulating = False
    
    
    def update_rmse(self,rmse): # 1
        if self.start_accumulating == True:
            self.drag_rmse_terms[self.count-1][1] = rmse
            if self.count == len(self.drag_rmse_terms):
                self.order_ready = True
                self.count = 0
                self.start_accumulating = False
            else:
                self.order_ready = False
        else:
            pass
            

    def best_centroid(self):
        self.drag_rmse_terms.sort(key=lambda each_element: each_element[1])
        self.lowest_rmse = self.drag_rmse_terms[0][1]
        cent_list = []
        for a in self.drag_rmse_terms[:-1]:
            cent_list.append(a[0])
        self.centroid = np.mean(cent_list, axis=0) 
        #print ("centroid: ", np.shape(centroid)) 
        return self.centroid


    def run_NM(self,alpha,gamma,sigma,rho): # 3
        self.sigma = sigma
        self.gamma = gamma
        self.rho = rho
        self.alpha = alpha
        if self.order_ready == True:
            self.best_centroid()
            if self.p1 == True:
                self.reflection_f(self)
            elif self.p3 == True:
                self.expansion_f(self)
            elif self.p5 == True:
                self.contraction_f(self)
            elif self.p7 == True:
                self.shrink_f(self)
        else:
            self.start_accumulating = True
            self.count += 1
            return self.drag_rmse_terms[self.count-1][0]            
                

    def reflection_f(self):
        self.reflection = self.centroid + self.alpha*(self.centroid - self.drag_rmse_terms[-1][0])
        self.p2 = True
        return self.reflection
    
    
    def expansion_f(self):
        self.expansion = self.centroid + self.gamma*(self.reflection - self.centroid)
        self.p4 = True
        return self.expansion
    

    def contraction_f(self):
        if self.old_reflection_rmse < self.drag_rmse_terms[-1][1]:
            self.contraction = self.centroid + self.rho*(self.reflection - self.centroid)
            self.p61 = True
        elif self.old_reflection_rmse >= self.drag_rmse_terms[-1][1]:
            self.contraction = self.centroid + self.rho*(self.drag_rmse_terms[-1][0] - self.centroid)
            self.p62 = True
        return self.contraction
    

    def shrink_f(self):
        for i in range(len(self.drag_rmse_terms)-1):
            self.drag_rmse_terms[i+1][0] = self.drag_rmse_terms[0][0] + self.sigma*(self.drag_rmse_terms[i+1][0] - self.drag_rmse_terms[0][0])
        self.order_ready = False
        self.count = 2
        self.p7 = False
        self.start_accumulating = True
        self.p1 = True
        return self.drag_rmse_terms[1][0] # back to square one


    def centroid_test(self,rmse): # 2
        if self.p2 == True: # reflection phase comparison
            if rmse > self.lowest_rmse and rmse < self.drag_rmse_terms[-2][1]:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.reflection # update w reflection
                self.p2 = False
            elif rmse < self.lowest_rmse:
                self.old_reflection_rmse = rmse
                self.p3 = True # activate expansion
                self.p1 = False
                self.p2 = False
            elif rmse > self.drag_rmse_terms[-2][1]:
                self.old_reflection_rmse = rmse
                self.p5 = True
                self.p1 = False
                self.p2 = False
                self.p3 = False
                self.p4 = False
        elif self.p4 == True: # expansion phase comparison
            if rmse < self.old_reflection_rmse:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.expansion # update w expansion
                self.p4 = False
                self.p3 = False
                self.p1 = True
                self.p2 = False
            elif rmse > self.old_reflection_rmse:
                self.drag_rmse_terms[-1][1] = self.old_reflection_rmse
                self.drag_rmse_terms[-1][0] = self.reflection # update w reflection
                self.p4 = False
                self.p3 = False
                self.p1 = True
                self.p2 = False
        elif self.p61 == True: # contraction phase 1 comparison
            if rmse < self.old_reflection_rmse:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.contraction # update w contraction
                self.p61 = False
                self.p5 = False
                self.p1 = True
                self.p2 = False
            else:
                self.p61 = False
                self.p5 = False 
                self.p7 = True # activate shrink
                self.p2 = False
        elif self.p62 == True: # contraction phase 2 comparison
            if rmse < self.drag_rmse_terms[-1][1]:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.contraction # update w contraction
                self.p62 = False
                self.p5 = False
                self.p1 = True
                self.p2 = False
            else:
                self.p62 = False
                self.p5 = False 
                self.p7 = True # activate shrink
                self.p2 = False            
        else:
            pass

    





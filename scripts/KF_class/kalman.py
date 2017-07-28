#! /usr/bin/python

import numpy as np
#from scipy import linalg
import math
from math import pow,atan2,sqrt,cos,sin,asin,pi


class KF():
      
      # Init fuction for KF
      def __init__(self, A, B, C, R, Q, K):

            self.A = A
            self.B = B
            self.C = C
            self.R = R
            self.Q = Q
            self.K = K
            self.I = np.identity(len(A))

            #predicted state

            pmt_1 = 0.0
            pmt_2 = 0.0
            pmt_3 = 0.0
            pmt_4 = 0.0

            self.pmt_ = [[pmt_1],
                        [pmt_2],
                        [pmt_3],
                        [pmt_4]]

            #corrected state

            mt1 = 0.0
            mt2 = 0.0
            mt3 = 0.0
            mt4 = 0.0

            self.mt = [[mt1],
                  [mt2],
                  [mt3],
                  [mt4]]


            #predicted covariance

            pSt_1 = 0.0
            pSt_2 = 0.0
            pSt_3 = 0.0
            pSt_4 = 0.0

            self.pSt_ = [[pSt_1,0,0,0],
                        [0,pSt_2,0,0],
                        [0,0,pSt_3,0],
                        [0,0,0,pSt_4]]

            #corrected covariance

            St1 = 0.0
            St2 = 0.0
            St3 = 0.0
            St4 = 0.0

            self.St = [[St1,0,0,0],
                        [0,St2,0,0],
                        [0,0,St3,0],
                        [0,0,0,St4]]

            self.z_last = [[0],[0],[0]]
            self.flag1 = 0
            self.flag2 = 0
            self.cont = 0


      # This function computes the kalman filter its arguments are:
      # mt_ : is the previous state as a result of KF
      # St_ : is the previous Coveriance as a result of KF
      # ut  : is the control input vector 
      # zt  : is the vector of measurments from sensors
      # Q   : is the covariance  matrix from the sensors
      # dt  : is the sample peroid in seconds

      def KF_compute(self, mt_, St_, ut, zt, Q, dt):
            #control matrix
            self.B = [[dt,0,0],
                 [0,dt,0],
                 [0,0,dt]]	
            #covariance matrix from sensors
            self.Q = Q

            # if zt == self.z_last:
            #       self.falg1 = 1
            # if self.flag1

            #predict the state Ax+Bu = x' 
            self.pmt_ = np.add( np.dot( self.A , mt_ ) , np.dot( self.B , ut ) )
            #predict the covariance matrix
            self.pSt_ = np.add( np.dot( np.dot( self.A , St_ ) , np.transpose(self.A) ) , self.R )

            # Step 2: Belief compute and corrective
            # if not (self.flag == 10):
                  #determinate the Kalman gain
            self.K = np.dot( np.dot( self.pSt_ , np.transpose(self.C)) , np.linalg.inv( np.add( np.dot( np.dot( self.C , self.pSt_ ) , np.transpose(self.C) ) , self.Q)))
                  # cont+=cont
                  # print("adentro")
            # else:
                  # print("afuera")
            #determinate the corrected state
            self.mt = np.add( self.pmt_ , np.dot( self.K , np.subtract( zt , np.dot( self.C , self.pmt_ ) ) ) )
            #determinate the corrected covariance matrix
            self.St = np.dot( np.subtract( self.I , np.dot( self.K , self.C ) ) , self.pSt_ )
            
            # self.z_last = zt
            #retun KF results: state vector and Covariance matrix
            return [self.mt, self.St]

            #this fuction is used to test the model
      def compute(self, mt_, ut, dt):
            #control matrix
            self.B = [[dt,0,0],
                      [0,dt,0],
                      [0,0,dt]]
            self.mt = np.add( np.dot( self.A , mt_ ) , np.dot( self.B , ut ) )
	      
            return [self.mt, self.St]


#if __name__ == '__main__':



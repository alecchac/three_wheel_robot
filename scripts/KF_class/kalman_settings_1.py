#! /usr/bin/python

#-------------Kalman definitions -----------------

# covariance matrices
#Process covariance
#si decrementas suaviza pero es mas lenta su respuesta

R1 = 5
R2 = 5
R3 = 5

R = [[R1,0,0],
	[0,R2,0],
	[0,0,R3]]

#Measurment covariance
#entre mas pequeno le cree mas a la medida

Q1 = 1
Q2 = 1
Q3 = 1


Q = [[Q1,0,0],
     [0,Q2,0],
     [0,0,Q3]]

# kalman gains definition

k_1 = 0.0
k_2 = 0.0
k_3 = 0.0


K = [[k_1,0,0],
     [0,k_2,0],
     [0,0,k_3]]

import numpy as np
import matplotlib.pyplot as plt


theta=0
domega=0
omega=0
J=0.1
K=8.72
R=0.3
I=0
V=24
L=0.005

D=0.15
dI=0

w=120
Ea=0
B=0.1

distance=100



t=1			#time in seconds
dt=0.000001		#step size in seconds

y=np.zeros(t/dt)
y2=np.zeros(t/dt)
y3=np.zeros(t/dt)
x=np.zeros(t/dt)
p=np.zeros(t/dt)


for i in range(0,int(t/dt)):

	# w=20*(distance-(theta*D/2))



	# V=24*(w-omega)

	# if V>24:
	# 	V=24
	# if V<-24:
	# 	V=-24

	T=K*I-B*omega
	Ea=omega/K

	domega=T/J
	omega=omega+domega*dt
	theta=theta+omega*dt

	#I=(V-Ea)/R

	dI = (V - Ea - R*I) / L
	I = I + dI*dt

	y[i]=omega
	y2[i]=I
	y3[i]=V
	x[i]=i*dt
	p[i]=theta*D/2



print omega

plt.subplot(311)
plt.grid(True)
plt.plot(x,y)
plt.ylabel('Velocity [rad/s]')
plt.subplot(312)
plt.grid(True)
plt.plot(x,y2,x,y3)
plt.ylabel('Current [A]')
plt.subplot(313)
plt.grid(True)
plt.plot(x,p)
plt.ylabel('Position[M]')
plt.xlabel('Time [seconds]')
plt.show()
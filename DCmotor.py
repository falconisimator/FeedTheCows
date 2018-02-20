import numpy as np
import matplotlib.pyplot as plt


class DC_Motor:

	def __init__(self,K,R,L,B=0,J=0,F=0):
		self.theta=0
		self.domega=0
		self.omega=0
		self.T=0
		self.I=0

		self.K=K 	#Motor Constant
		self.B=B 	#Viscous damping coefficient
		self.J=J	#Motor Moment of inertia
		self.R=R 	#Motor Resistance
		self.L=L
		self.F=F
		self.Fa=0

	def update(self,V,dt):

		if abs(self.K*self.I-self.B*self.omega)>self.F:
			self.Fa=self.F*np.sign(self.omega)
		else:
			self.Fa=self.K*self.I-self.B*self.omega

		self.T=self.K*self.I-self.B*self.omega-self.Fa
		self.Ea=self.omega/self.K

		self.domega=self.T/self.J
		self.omega=self.omega+self.domega*dt
		self.theta=self.theta+self.omega*dt

		self.dI = (V - self.Ea - self.R*self.I) / self.L
		self.I = self.I + self.dI*dt

		return(self.omega,self.I)

class Motor_Controller:

	def __init__(self):
		self.sat=0

	def P(self,target,current,k):

		self.e = target - current
		self.p=self.e*k

		if self.sat==1:
			if self.p>self.upper:
				self.p=self.upper
			if self.p<self.lower:
				self.p=self.lower

		return self.p

	def saturate(self,upper,lower):
		self.upper=upper
		self.lower=lower
		self.sat=1



	
m=2200
V=24

D=0.15

t=20			#time in seconds
dt=0.0001		#step size in seconds

y=[]
y2=[]
y3=[]
x=[]
p=[]


Motor1=DC_Motor(8.72,0.3,0.001,B=0.1,J=0.1+m*(D/2)**2)
C1=Motor_Controller()
C1.saturate(24,-24)


for i in range(0,int(t/dt)):

	V=C1.P(200,Motor1.omega,24)

	Motor1.update(V,dt)

	y.append(Motor1.omega)
	y2.append(Motor1.I)
	y3.append(V)
	x.append(i*dt)
	p.append(Motor1.theta)

print Motor1.omega


plt.subplot(411)
plt.grid(True)
plt.plot(x,y)
plt.ylabel('Velocity [rad/s]')
plt.subplot(412)
plt.grid(True)
plt.plot(x,p)
plt.ylabel('Position[rad]')
plt.subplot(413)
plt.grid(True)
plt.plot(x,y2)
plt.ylabel('Current [A]')
plt.subplot(414)
plt.grid(True)
plt.plot(x,y3)
plt.ylabel('Voltage [V]')
plt.xlabel('Time [seconds]')
plt.show()
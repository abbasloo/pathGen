from navpy import ecef2lla, lla2ecef
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt
import math
import csv


d = 40
#lla = [lat, lon, alt] #[degree, degree, meter]
#ecef = [x, y, z] #[meter, meter, meter]
					
				
def Cylinder(location, R, H_min, H_max):	
	path = []
	for i in range(d):
		for k in range(d):
			p = [R*math.cos(2*np.pi/d*i)+location[0], 
			     R*math.sin(2*np.pi/d*i)+location[1],
                             (H_max-H_min)/d*k + H_min+location[2]]
			path.append(p)
	return np.asarray(path)

def Sphere(location, R, H_min, H_max):	
	path = []
	for i in range(d):
		for j in range(d):
			if R*math.cos(np.pi/d*i) > H_min:
				if R*math.cos(np.pi/d*i) < H_max:
					p = [R*math.sin(np.pi/d*i)*math.cos(2*np.pi/d*j)+location[0], 
					     R*math.sin(np.pi/d*i)*math.sin(2*np.pi/d*j)+location[1],
        	       	        	     R*math.cos(np.pi/d*i)+location[2]]
					path.append(p)
	return np.asarray(path)

def Plane(location, P1, P2, H_min, H_max):	
	P1 = lla2ecef(P1[0], P1[1], P1[2]) 
	P2 = lla2ecef(P2[0], P2[1], P2[2]) 
	P3 = [(P1[0]+P2[0])/2, (P1[1]+P2[1])/2, (P1[3]+P2[3])/2+20]	
	R = np.amax([np.linalg.norm(location[0:2]-P1[0:2],'fro'), np.linalg.norm(location[0:2]-P2[0:2])],'fro')
	P1 = P1 - location
	P2 = P2 - location
	P3 = P3 - location
	path = []
	for i in range(d):
		for j in range(d):
			for k in range(d):
				if R/d*k > H_min:
					if R/d*k < H_max:
						M = Np.array([[R/d*i-P1[0], R/d*j-P1[1], R/d*k-P1[2]],
							      [R/d*i-P2[0], R/d*j-P2[1], R/d*k-P2[2]],
							      [R/d*i-P3[0], R/d*j-P3[1], R/d*k-P3[2]]])
						if np.linalg.det(M) == 0:
							p = [R/d*i+location[0], R/d*j+location[1], R/d*k+location[2]]
							path.append(p)
	return np.asarray(path)

location = [50.949650, 6.933180, 30] #[lat, lon, alt] 
location_xyz = lla2ecef(location[0], location[1], location[2]) 
#print (location_xyz)

xyz = Sphere(location_xyz, 20.0, -2, +20)
#print (xyz)
lla = ecef2lla(xyz)

f = open('path.csv', 'wt')
writer = csv.writer(f)
writer.writerow( ('lat', 'lon', 'alt') )
for i in range(len(lla[0])):
        writer.writerow((lla[0][i], lla[1][i], lla[2][i]))
f.close()

for i in range(len(lla[0])):
	print ('lat', 'lon', 'alt')
	print (lla[0][i], lla[1][i], lla[2][i])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=10, marker='o')
ax.scatter(location_xyz[0], location_xyz[1], location_xyz[2], s=50, marker='*')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()



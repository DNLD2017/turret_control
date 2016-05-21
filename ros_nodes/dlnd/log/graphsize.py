import matplotlib.pyplot as plt
import csv

x = []
h = []
s = []
v = []
xx = []
yy = []
size = []

with open('test.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(int(row[0]))
        h.append(int(row[1]))
	s.append(int(row[2]))
	v.append(int(row[3]))
	xx.append(int(row[4]))
	yy.append(int(row[5]))
        size.append(int(row[6]))

plt.ion()
#plt.plot(x[0:1],h[0:1], label='h')
#plt.plot(x[0:1],s[0:1], label='s')
#plt.plot(x[0:1],v[0:1], label='v')
plt.plot(x[0:1],size[0:1], 'y')
plt.xlabel('x') 
plt.ylabel('y')
plt.title('(x,y) Evolution')
plt.legend()
    
plt.draw()

j = 0
for i in x:
    #plt.plot(x[0:i],h[0:i], 'b', label='h')
    #plt.plot(x[0:i],s[0:i], 'g', label='s')
    #plt.plot(x[0:i],v[0:i], 'r', label='v')
    plt.plot(x[0:i],size[0:i], 'y')
    if j%3 ==0:
	plt.draw() 
    j = j+1 

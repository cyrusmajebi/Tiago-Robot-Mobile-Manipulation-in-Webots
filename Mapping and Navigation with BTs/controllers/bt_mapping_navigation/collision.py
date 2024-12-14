from skimage.draw import line_nd, random_shapes
import numpy as np
from matplotlib import pyplot as plt





def IsPathOpen(map,a,b):
    if not (map[round(b[0])][round(b[1])] == False):
        return False
        
    (x,y) = line_nd(a,b,integer=True)
    f_count = 0
    
    for i in range(len(x)):
        print()
        if map[x[i]][y[i]] == False:
            # plt.plot(y[i], x[i], 'w.')
            # plt.show()
            # plt.pause(0.000001)
            pass
        else:
            # plt.plot(y[i], x[i], 'r.')
            # plt.show()
            # plt.pause(0.000001)
            f_count += 1
            
    if f_count > 0:
        #print("Not obstacle free")
        return False   
        
    #print("Obstacle free")
    return True









#map = np.ones((200,300))*255
#map, labels = random_shapes((200,300),6,5,num_channels=1)

#plt.imshow(map)

# plt.figure(facecolor='purple')
# plt.axes().set_facecolor("purple")
#plt.show()

# print(map.shape)

# print(map[100][50])

# plt.imshow(map)
# plt.show()

# l = line_nd((0,1,2),(5,6,7))
# print(l)

# a = (10,10)
# b = (200,190)

# (x,y) = line_nd((10,10), (15,20),integer=True)
# print((x,y))   
# print((x[5],y[5]))  
    
    
#IsPathOpen(map,a,b)
    
    
    
    
    
    
    
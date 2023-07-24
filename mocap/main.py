from pseyepy import Camera, Display, Stream
import time
import matplotlib.pyplot as plt
import numpy as np


c = Camera(fps=120, resolution=Camera.RES_SMALL)
Display(c)

#s = Stream(c, file_name='video.avi', display=True) # begin saving data to files

# when finished, close the stream
#s.end()

# for i in range(0, 100):
#     print(i)
#     c.read()

# fig, (ax1, ax2) = plt.subplots(1, 2)
# frame, timestamp = c.read()
# a = ax1.imshow(frame[0])
# b = ax2.imshow(frame[1])

# dataa = []
# datab = []
# for i in range(0, 30):
#     frame, timestamp = c.read()
#     a.set_data(np.fliplr(np.flipud(frame[0])))
#     b.set_data(frame[1])
#     dataa.append(frame[0].copy())
#     datab.append(frame[1].copy())
#     plt.pause(.0001)
#     plt.draw()

# print(len(dataa))

# for i in range(len(dataa)):
#     plt.imsave(f'{i}a.png', dataa[i])
#     plt.imsave(f'{i}b.png', datab[i])
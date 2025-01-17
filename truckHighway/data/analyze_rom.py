import numpy as np
import matplotlib.pyplot as plt

import sys,os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
# Add the parent directory of 'models' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


# Load the data
vir_data = np.genfromtxt(project_root+'/data/virstate.csv', delimiter=',')
real_data = np.genfromtxt(project_root+'/data/realstate.csv', delimiter=',')

# plot the data
plt.figure(figsize=(15,10))
plt.subplot(3,1,1)
plt.plot(vir_data[:,0], vir_data[:,3], label='heading ROM')
plt.plot(real_data[:,0], real_data[:,3], label='heading Chrono')
plt.ylabel('Heading (rad)')
plt.grid()
plt.legend()
plt.subplot(3,1,2)
plt.plot(vir_data[:,0], vir_data[:,4], label='speed ROM')
plt.plot(real_data[:,0], real_data[:,4], label='speed Chrono')
plt.ylabel('Speed (m/s)')
plt.grid()
plt.legend()
plt.subplot(3,1,3)
plt.plot(vir_data[:,0], vir_data[:,5], label='acceleration ROM')
plt.plot(real_data[:,0], real_data[:,5], label='acceleration Chrono')
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.grid()
plt.legend()
plt.tight_layout()
plt.show()

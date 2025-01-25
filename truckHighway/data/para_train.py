import numpy as np
import matplotlib.pyplot as plt

import sys,os
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
# Add the parent directory of 'models' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

train_data = np.genfromtxt(project_root+'/data/training.csv', delimiter=',')

vel = train_data[:,4]
throttle = train_data[:,6]
braking = train_data[:,8]
acc = train_data[:,5]

# # get index that throttle*braking > 1e-3
# idx = np.where(throttle*braking < 1e-6)
# vel = vel[idx]
# throttle = throttle[idx]
# braking = braking[idx]
# acc = acc[idx]

print(f"before filtering: {train_data.shape[0]}, after filtering: {vel.shape[0]}")

alphat = throttle - braking
# feature=[1, vel, vel*alpha_t, vel^2 * alpha_t, vel^3 * alpha_t]
feature = np.column_stack((np.ones_like(vel), vel, vel*alphat, vel**2 * alphat))

brake_effect_coef = 0.9

# fit data into form acc = F(vel,throttle)  - brake_effect_coef * vel * braking
# fit F into polynomial form

# Fit the polynomial regression model
model = LinearRegression().fit(feature, acc)
# feature = [vel, throttle]
# acc = model.predict(poly.fit_transform([feature]))[0]

# Extract the coefficients
F = model.coef_
print(F)

# The coefficients correspond to the polynomial terms in the following order:
# [1, vel, throttle, vel^2, vel*throttle, throttle^2, vel^3, vel^2*throttle, vel*throttle^2, throttle^3]
# Therefore, the formula for F(vel, throttle) is:
# F(vel, throttle) = 0.874514302 + 9.68850068*vel - 0.0260284792*throttle - 1.43839874*vel^2 + 27.6230313*vel*throttle
#                    + 0.000277699868*throttle^2 + 0.0182813753*vel^3 + 0.516546131*vel^2*throttle - 24.0527775*vel*throttle^2

# Generate synthetic acceleration data
syn_acc = model.predict(feature) 

plt.figure(figsize=(15,10))
plt.plot(acc, label='real acc')
plt.plot(syn_acc, label='syn acc')
plt.legend()
plt.show()

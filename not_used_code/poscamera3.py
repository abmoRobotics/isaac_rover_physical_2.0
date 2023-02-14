from scipy.spatial.transform import Rotation as R

r = R.from_euler('z', 90, degrees=True)
print(r.as_euler('xyz'))
print(r.inv().as_euler('xyz'))
import numpy as np

import matplotlib.pylab as plt

data = np.loadtxt('SF_LA_usgs_data.txt', skiprows=1, delimiter=",")

old_lon = None
old_lat = None

lons = set(data[:, 0])
lats = set(data[:, 1])

n_lons = len(lons)
n_lats = len(lats)

Longitude = np.empty((n_lons, n_lats))
Latitude = np.empty((n_lons, n_lats))
Elevation = np.empty((n_lons, n_lats))

i_lon = -1
i_lat = -1

old_lon = None
old_lat = None

for i, (lon, lat, z) in enumerate(data):
    if old_lon != lon:
        old_lon = lon
        i_lon += 1

    if old_lat != lat:
        old_lat = lat
        i_lon = 0 # need to reset the column counter
        i_lat += 1  # move to the next row

    Longitude[i_lat, i_lon] = lon
    Latitude[i_lat, i_lon] = lat
    Elevation[i_lat, i_lon] = z

np.savez('usgs_data', xx=Longitude, yy=Latitude, zz=Elevation)

fig, ax = plt.subplots()
contour_data = ax.contourf(Longitude, Latitude, Elevation)
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
fig.colorbar(contour_data)
plt.show()
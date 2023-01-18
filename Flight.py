# DRONE SHOULD BE FACING DUE NORTH (0/360) UPON TAKEOFF

# FIXES Needed
#   Return to home is not being output

from pykml import parser
import os
import sys
import glob
import pyproj
from djitellopy import Tello
import cv2
import time
from threading import Thread
from datetime import datetime
import logging
import subprocess

# Setup Logging
logging.basicConfig(filename='DronePatrol.log', encoding='utf-8', format='%(asctime)s %(message)s', level=logging.DEBUG)
now = datetime.now()
logging.info('========= SCRIPT START =========')

# Locate KML file and ensure there is only one
kmlPath = "./"
kmlCount = len(glob.glob1(kmlPath,"*.kml"))
if kmlCount < 1:
    sys.exit("Exiting. No KML file found in the root directory.")
    logging.error('Exiting. No KML file found in the root directory.')
elif kmlCount > 1:
    sys.exit("Exiting. You may only have one KML file in the root directory.")
    logging.error('Exiting. You may only have one KML file in the root directory.')
else:
    for x in os.listdir():
        if x.endswith(".kml"):
            kml_file = x

# Open KML and start parsing
with open(kml_file) as f:
    doc = parser.parse(f)
root = doc.getroot()
folder = root.Document.Folder

# Create empty array for coords
arrkey = 0
coords = {}

logging.debug ('--------- KML Extracted Coordinates ---------')

for pm in (folder.Placemark):
    # Extract Coordinate Name
    pmname = pm.name
    
    # Extract coordinates
    pmcoord = pm.Point.coordinates.text
    
    # Coordinate string cleanup
    pmcoord2 = pmcoord.replace("\n","")
    pmcoord3 = pmcoord2.replace(" ","")
    pmcoord4 = pmcoord3.replace(",0","")
    
    # Get some substring stuff ready
    delim = pmcoord4.index(',')
    last = len(pmcoord4)
    
    # Extract Longitude
    Long = pmcoord4[0:delim]

    # Extract Latitude
    LatStart = delim + 1
    Lat = pmcoord4[LatStart:last]

    # Add coordinates to array
    coords[arrkey] = {}
    coords[arrkey]['wp'] = pmname
    coords[arrkey]['lat'] = Lat
    coords[arrkey]['lng'] = Long

    debuglog = arrkey, pmname, Lat, Long
    logging.debug(debuglog)

    # Increment Array Key
    arrkey += 1

logging.debug('--------- Coords Array ---------')
debuglog = coords
logging.debug(debuglog)

# Begin coordinate array loops to find distances and bearings
# First section determines the number of loops needed based on
# count of waypoints in KML file.
numcoords = len(coords)

coordcheck = numcoords % 2

if coordcheck == 0:
    iterations = (numcoords / 2) + 2
else:
    iterations = (numcoords / 2) + 1.5

itrint = int(iterations)

itct = 0

logging.debug('--------- Waypoint Data ---------')
# Loop through coordinate pairs and store data in array
waypoints = {}
while itct <= itrint:
    lat1 = coords[itct]['lat']
    lng1 = coords[itct]['lng']
    loc1 = coords[itct]['wp']
    
    # Logic to return to waypoint 1 (base) at the end
    if itct + 2 > numcoords:
        lat2 = coords[0]['lat']
    else:
        lat2 = coords[itct + 1]['lat']
    
    if itct + 2 > numcoords:
        lng2 = coords[0]['lng']
        
    else:
        lng2 = coords[itct + 1]['lng']
    if itct + 2 > numcoords:
        loc2 = coords[0]['wp']
    else:
        loc2 = coords[itct + 1]['wp']
    
    # Get distance and bearing values between waypoints
    geodesic = pyproj.Geod(ellps='WGS84')
    fwd_azimuth,back_azimuth,distance = geodesic.inv(lng1, lat1, lng2, lat2)

    print(loc1,"->",loc2,"Distance:",distance * 100,"cm Bearing:",fwd_azimuth)
    
    debuglog = loc1,"->",loc2,"Distance:",distance * 100,"cm Bearing:",fwd_azimuth
    logging.debug(debuglog)
    
    # Store in array
    waypoints[itct] = {}
    waypoints[itct]['loc1'] = loc1
    waypoints[itct]['loc2'] = loc2
    waypoints[itct]['distance'] = distance * 100
    if fwd_azimuth < 0:
        waypoints[itct]['bearing'] = fwd_azimuth + 360
    else:
        waypoints[itct]['bearing'] = fwd_azimuth

    debuglog = itct, loc1, loc2, distance * 100, fwd_azimuth
    logging.debug(debuglog)

    itct += 1

logging.debug('--------- Waypoint Array No Rotation ---------')
debuglog = waypoints
logging.debug(debuglog)

logging.debug('--------- Waypoint Rotation Data ---------')

# Calculate roatation degrees CW CCW for drone at each waypoint
numwp = len(waypoints)
wpct = 0
while wpct < numwp:
    if wpct == 0:
        headchg = 0 - waypoints[wpct]['bearing']
        if headchg < -180:
            headchg = 360 + headchg
            rot = "CCW"
            waypoints[wpct]['rotation'] = headchg
            waypoints[wpct]['rotdir'] = rot
        else:
            headchg = 360 - headchg
            rot = "CW"
            waypoints[wpct]['rotation'] = headchg
            waypoints[wpct]['rotdir'] = rot
    else:
        headchg = waypoints[wpct]['bearing'] - waypoints[wpct - 1]['bearing']
        if headchg < 0:
            headchg = -headchg
            rot = "CCW"
            waypoints[wpct]['rotation'] = headchg
            waypoints[wpct]['rotdir'] = rot
        else: 
            rot = "CW"
            waypoints[wpct]['rotation'] = headchg
            waypoints[wpct]['rotdir'] = rot
    print(headchg,rot)
    
    debuglog = wpct,headchg,rot
    logging.debug(debuglog)
    
    wpct += 1

print(waypoints)

logging.debug('--------- Waypoint Array With Rotation ---------')
debuglog = waypoints
logging.debug(debuglog)


 
print("now =", now)

# Connect to tello
logging.info('========= CONNECTING TO TELLO =========')
Tello = Tello()
Tello.connect()

# Define Status Variables
batt = 'Battery %:',Tello.get_battery()
accx = 'Acceleration X:',Tello.get_acceleration_x()
accy = 'Acceleration Y:',Tello.get_acceleration_y()
accz = 'Acceleration Z:',Tello.get_acceleration_z()
baro = 'Barometer:',Tello.get_barometer
tofd = 'TOF Distance (cm):',Tello.get_distance_tof()
tof = 'Flight Time (s):',Tello.get_flight_time()
height = 'Height (cm):',Tello.get_height()
temp = 'Temp (C):',Tello.get_temperature()
temph = 'Temp High (C):',Tello.get_highest_temperature()
templ = 'Temp Low (C):',Tello.get_lowest_temperature()
pitch = 'Pitch:',Tello.get_pitch()
roll = 'Roll:',Tello.get_roll()
yaw = 'Yaw:',Tello.get_yaw()
spdx = 'Speed X:',Tello.get_speed_x()
spdy = 'Speed Y:',Tello.get_speed_y()
spdz = 'Speed Z:',Tello.get_speed_z()
udpvid = "UDP Video Address:",Tello.get_udp_video_address()

# Get Tello System Statuses
logging.info('--------- Tello Pre-Flight Check ---------')
logging.info('SYSTEM STATS:')
logging.info(batt)
logging.info(accx)
logging.info(accy)
logging.info(accz)
logging.info(baro)
logging.info(tofd)
logging.info(tof)
logging.info(height)
logging.info(temp)
logging.info(temph)
logging.info(templ)
logging.info(pitch)
logging.info(roll)
logging.info(yaw)
logging.info(spdx)
logging.info(spdy)
logging.info(spdz)
logging.info('NETWORK INFO')
logging.info(udpvid)

print('--------- Tello Pre-Flight Check ---------')
print('SYSTEM STATS:')
print(batt[0],batt[1])
print(accx[0],accx[1])
print(accy[0],accy[1])
print(accz[0],accz[1])
print(baro[0],baro[1])
print(tofd[0],tofd[1])
print(tof[0],tof[1])
print(height[0],height[1])
print(temp[0],temp[1])
print(temph[0],temph[1])
print(templ[0],templ[1])
print(pitch[0],pitch[1])
print(roll[0],roll[1])
print(yaw[0],yaw[1])
print(spdx[0],spdx[1])
print(spdy[0],spdy[1])
print(spdz[0],spdz[1])
print('NETWORK INFO')
print(udpvid[0],udpvid[1])

# Start video stream
logging.info('--------- Starting Video Stream ---------')
print('--------- Starting Video Stream ---------')
Tello.streamoff()
Tello.streamon()
p0 = subprocess.Popen(['cmd','/C','C:\\ffmpeg\\bin\\ffmpeg.exe  -i udp://0.0.0.0:11111 -preset ultrafast -vcodec libx264 -tune zerolatency -b 900k -f h264 udp://127.0.0.1:5000 test2.avi'])


time.sleep(20)
p0.terminate




Tello.streamoff()
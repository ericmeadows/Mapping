import pandas as pd
import numpy as np
import sys
import os.path
import time
tic = time.clock()
tic2=tic
# import pdb

# Earth Geometry Parameters
hL = 6.13/3.0;      # Point heading threshold [m]; above this, measurement is rejected
dL = 20;            # Point separation [m]; above this, points are not "neighbors"
aF = 4;             # GPS accuracy threshold [m]; above this, measurement is rejected 

maxSegLen = 160.934

# Search threshold to minimize operations
latiF = 0.0005;     # Latitude delta ~50 meters
lngiF = 0.0005;     # Longitude delta ~50 meters

r = 6378137;

# For resuming
lenLastId = 0
fname = "/home/emeadows/telemetry/lastClusterId_testLanes_examineAndDecide.txt"
if os.path.isfile(fname):
    f = open(fname,'r')
    lenLastId = int(f.readline())
    f.close()

# Data input and output files
lanesFile = ("/home/emeadows/telemetry/testLanes.csv")
lanesOutFile = ("/home/emeadows/telemetry/testLanes_v3.csv")
print "loaded"

# If restart is desired, enable
# lenLastId = 0

if lenLastId == 0:
    lane_data = pd.read_csv("C:/Users/emeadows/Documents/ADAS/telemetryProject/ContinualProcessing/ElCamino_ChateauDrive_HillsdaleBlvd.csv")

    lane_data['lane_info'] = lane_data['lane_info'].astype('str')

    lane_data = lane_data[lane_data['lane_info'].str.len() == 9]
    lane_data = lane_data[(lane_data['gps_accuracy'] <= aF)]
    lane_data.reset_index(inplace=True)

    # Conversion from string-stored values to integers
    lane_data['RoadClass'] = lane_data['lane_info'].str[0].astype('int')
    lane_data['LLaneType'] = lane_data['lane_info'].str[1].astype('int')
    lane_data['LLaneQual'] = lane_data['lane_info'].str[2].astype('int')
    lane_data['LLaneColo'] = lane_data['lane_info'].str[3].astype('int')
    lane_data['LLaneCros'] = lane_data['lane_info'].str[4].astype('int')
    lane_data['RLaneType'] = lane_data['lane_info'].str[5].astype('int')
    lane_data['RLaneQual'] = lane_data['lane_info'].str[6].astype('int')
    lane_data['RLaneColo'] = lane_data['lane_info'].str[7].astype('int')
    lane_data['RLaneCros'] = lane_data['lane_info'].str[8].astype('int')
    lane_data.drop('lane_info', axis=1, inplace=True)

    lane_data['clusterId'] = lane_data.index
else:
    lane_data = pd.read_csv(lanesFile)


evaluatedSet = []
toEvaluateSet = []
prioritySet = pd.DataFrame()

cdx = 0

maxSize = len(lane_data)
lastHundred = 0
print "examining"
# try:
for item, noo in lane_data[lenLastId::].iterrows():
    tempEvaluatedSet = []
    if item in evaluatedSet:
        continue
    toEvaluateSet.append(item)
    if lane_data['gps_accuracy'].iloc[item] > aF:
        lane_data.iloc[item,('clusterId')] = 0
        evaluatedSet += [item]
        continue
    while True:
        skipSet = pd.DataFrame()
        cdx += 1
        # used for Max distance = 0.1 mile
        minLat = noo['latitude']
        minLong = noo['longitude']
        maxLat = noo['latitude']
        maxLong = noo['longitude']
        # return to normal
        while True:
            if len(toEvaluateSet) == 0:
                divVal = floor(len(tempEvaluatedSet)/100)
                if divVal > lastHundred:
                    lastHundred = divVal
                    lane_data.to_csv("/home/emeadows/Data/newLaneData.csv")
            dex = toEvaluateSet.pop(0)
            irow = lane_data.loc[dex]
            evaluatedSet += [dex]
            tempEvaluatedSet  += [dex]
            # GPS within searchable region via simple boolean
            latiVal = (lane_data['latitude'] - irow['latitude']) < latiF
            lngiVal = (lane_data['longitude'] - irow['longitude']) < lngiF
            headVal = ((lane_data['heading'] - irow['heading']) > (360 - hL)) | ((lane_data['heading'] - irow['heading']) < (0 + hL))
            # Join all conditions and make sure points have not been currently skipped, or evaluated
            sublist = set(lane_data[(latiVal & lngiVal & headVal)].index.tolist()) - set(skipSet.index.tolist()) - set(toEvaluateSet) - set(evaluatedSet)
            currentPoints = lane_data.loc[sublist]
            # Calculate distance from current evaluation point
            latiDelta = np.radians((currentPoints['latitude'] - irow['latitude'])/2)
            longDelta = np.radians((currentPoints['longitude'] - irow['longitude'])/2)
            distDelta = 2 * r * np.arcsin( np.sqrt( np.square(np.sin(latiDelta)) + np.multiply( np.multiply( np.cos(currentPoints['latitude']), np.cos(irow['latitude']) ), np.square( np.sin( longDelta ))) ))
            # Determine if distance to polygon extrema for current segment exceeds 0.1 mile
            latiDelta = np.radians((currentPoints['latitude'] - minLat)/2)
            longDelta = np.radians((currentPoints['longitude'] - minLong)/2)
            distMin = 2 * r * np.arcsin( np.sqrt( np.square(np.sin(latiDelta)) + np.multiply( np.multiply( np.cos(currentPoints['latitude']), np.cos(irow['latitude']) ), np.square( np.sin( longDelta ))) ))
            latiDelta = np.radians((currentPoints['latitude'] - maxLat)/2)
            longDelta = np.radians((currentPoints['longitude'] - maxLong)/2)
            distMax = 2 * r * np.arcsin( np.sqrt( np.square(np.sin(latiDelta)) + np.multiply( np.multiply( np.cos(currentPoints['latitude']), np.cos(irow['latitude']) ), np.square( np.sin( longDelta ))) ))
            inSegment = (distDelta < dL) & (distMin < maxSegLen) & (distMax < maxSegLen)
            
            # Removal of points that are now inside the segment
            if len(prioritySet) > 0:
                prioritySet = prioritySet[(set(prioritySet.index) - set(distDelta[inSegment].index))]
            
            outSegment = (distDelta < dL) & ((distMin > maxSegLen) | (distMax > maxSegLen))
            if outSegment.sum() > 0
                maxDist = pd.concat([distMin[outSegment],distMax[outSegment]],axis=1).max(axis=1)
                if len(prioritySet.index) == 0:
                    prioritySet = maxDist
                else:
                    prioritySet = prioritySet.append(maxDist[set(maxDist.index) - set(prioritySet.index)])
                if len(skipSet.index) == 0:
                    skipSet = maxDist
                else:
                    skipSet = skipSet.append(maxDist[set(maxDist.index) - set(skipSet.index)])
            
            # Update to segment extrema
            minLat = min(minLat,lane_data.loc[distDelta[inSegment].index,('latitude')].min())
            minLong = min(minLong,lane_data.loc[distDelta[inSegment].index,('longitude')].min())
            maxLat = max(maxLat,lane_data.loc[distDelta[inSegment].index,('latitude')].max())
            maxLong = max(maxLong,lane_data.loc[distDelta[inSegment].index,('longitude')].max())

            toEvaluateSet += [i for i in (set(distDelta[inSegment].index) - set(toEvaluateSet) - set(evaluatedSet))]
            if len(distDelta[(distDelta < dL)].index) != 0:
                lane_data.loc[distDelta[inSegment].index,('clusterId')] = cdx
            sys.stdout.write("\r{0} / {1} / {2}:  cdx - {3}, {4} of {5} items;\t{6}\t{7}\t{8}\t{9}".format( len(evaluatedSet),
                                                                                                        (lane_data['clusterId'] == cdx).sum(),
                                                                                                        len(lane_data),
                                                                                                        cdx,
                                                                                                        len(toEvaluateSet),
                                                                                                        inSegment.sum(),
                                                                                                        minLat,
                                                                                                        minLong,
                                                                                                        maxLat,
                                                                                                        maxLong))
            sys.stdout.flush()

        indexRow = item
        break
        if len(prioritySet) == 0:
            break
        else:
            prioritySet.sort()
            noo = lane_data[prioritySet.pop(prioritySet.index[0])]
        toc = time.clock()
        print "\n{0}".format(toc-tic2)
        tic2 = time.clock()
toc = time.clock()
print "\n{0}".format(toc-tic)
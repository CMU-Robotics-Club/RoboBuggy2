import json

'''
To run this file, make sure to follow the format of all location logs within a folder called loc_logs.
Each file should be of the format {loc_name}_{pixelX}_{pixelY} and contain NMEA information of
GNGGA format with the latitude and longitude. This script will take the information from UConnect
and convert it into a readable json called landmarks.json.

Additionally, we need an index of all location files wihtin file_pixel.csv. The format shoudl be identical to
loc_logs, in order to reconstruct file names.
'''


def cleanupFile(uncleanFileName):
    cleanFileName = uncleanFileName.replace(".", "_clean.")
    unclean = open(uncleanFileName, "r")
    clean = open(cleanFileName, "w")
    uncleanFile = unclean.readlines()
    counter = 0
    for line in uncleanFile:
        if ("GNGGA") in line and counter % 10 == 0:
            clean.write(line)
        counter += 1
    unclean.close()
    clean.close()
    return cleanFileName

#Parses the important data, store it in a dictionary
def parse_line(line) -> dict:
    lineDict = dict()
    data = line.split(",")
    lineDict["time"] = data[1]
    lineDict["lat"] = convertDegreeMinute(float(data[2]))
    lineDict["lon"] = -1 * convertDegreeMinute(float(data[4]))
    lineDict["quality"] = data[6]
    lineDict["num_sats"] = data[7]
    lineDict["active"] = False
    lineDict["key"] = lineDict["time"]
    return lineDict


def export_as_json(locations_data : [[dict]], filename) -> None:
    json_filename = filename.replace(".ubx", ".json")
    # print(loc_data)
    json_file = json.dumps(locations_data)
    f = open(json_filename, "w")
    f.write(json_file)

'''
Converting weird lat/long ddmm.degreeminute format by uconnect
into decimal degrees
'''
def convertDegreeMinute(degreeMinute):
    # print(degreeMinute)
    print(degreeMinute)
    if (degreeMinute > 6000):
        print(degreeMinute)
    degrees = degreeMinute//100
    minute_degree = degreeMinute % 100
    return (degrees) + minute_degree/60


if __name__ == "__main__":
    # TODO: make this dynamic?
    name = "/Users/mehulgoel/Documents/RoboBuggy2/rb_ws/src/buggy/paths/location-data/inside_curb.ubx"
    cleanFile = cleanupFile(name)

    timestamps_data = []
    file = open(cleanFile, "r")
    waypoints = file.readlines()
    for line in waypoints:
        timestamps_data.append(parse_line(line) )    
    
    file.close()   
    export_as_json(timestamps_data, cleanFile)
    




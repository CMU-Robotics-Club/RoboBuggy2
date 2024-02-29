import json

'''
To run this file, make sure to follow the format of all location logs within a folder called loc_logs.
Each file should be of the format {loc_name}_{pixelX}_{pixelY} and contain NMEA information of
GNGGA format with the latitude and longitude. This script will take the information from UConnect
and convert it into a readable json called landmarks.json.

Additionally, we need an index of all location files wihtin file_pixel.csv. The format shoudl be identical to
loc_logs, in order to reconstruct file names.
'''


coords = []
#Opens file, assumes said text file is inside loc_logs
def open_file(file_name: str) -> [[str]]:
    file_data : [[str]] = []
    with open("loc_logs/" + file_name, "r", encoding="utf-8", errors="ignore") as file:
        for line in file:
            #Each value in the textfile should be deliminated by a comma
            try:
                # Attempt to decode and process the line
                if ("$GNGGA" in line and "\n" not in line[:-2]):
                    file_data.append(line[:-1].split(","))

            except UnicodeDecodeError:
                pass
                #Handling Unicode Decoding Errors
    return file_data

#Parses the important data, store it in a dictionary
def parse_data(file_data : [[str]], pixel_coord, loc_name) -> dict:
    json_data : dict = {}
    midpoint_of_file = int(len(file_data)/2)
    if("margaret" in loc_name):
        print("Reached")
        pass
    for line in file_data[midpoint_of_file:]: #Iterating past midpoint until first GNGGA
        code = line[0] # 5 characters - tells us following line pattern

        """"
        GNGGA Format: GNGGA, timestamp, latitude (dd minute degrees), N/S, longitude (dd, minute degrees), E/W,
        """
        """
        We are assuming that a line in the middle of the file will be accurate to the position.
        This is because at the beginning and the end, we have initialization and other extraneous errors.
        This should be an effective way to mitigate these issues
        """
        if code == "$GNGGA": #the important one - gps data
            time = line[1] #UTC time

            #Removing extaneous case where given code but nothing else
            if line[2] == "":
                continue

            lat = convertDegreeMinute(float(line[2])) #Latitutde
            north_south = line[3] # north or south latitude
            lon = -1 * convertDegreeMinute(float(line[4])) #Longitude (negative bc always west)
            east_west = line[5] #East or West Longitude
            quality = line[6]
            num_sats = line[7]

            #THESE FIELDS AREN'T IN THE ERACER TEMPLATE - WILL BREAK? - MAYBE
            json_data.update({"pixel": pixel_coord})
            json_data.update({"time": time})
            json_data.update({"quality": quality})
            json_data.update({"num_sats": num_sats})

            #THESE FIELDS ARE IN ERACER BUT FIRST TWO ARE DUMMY VALS RN
            json_data.update({"key": loc_name + "-" + str(time)}) #TODO: FIX THE UNIQUE LOCATION TO BE UNIQUE
            json_data.update({"lat": lat})
            json_data.update({"lon": lon})
            json_data.update({"active": False})
            if("margaret" in loc_name): print(json_data)
            return json_data#EXIT AFER WE GRAB A SINGULAR WORKING LINE

def consolidate(locations_data : [[dict]], loc_data : [dict]) -> None:
    if (loc_data != None): #Removes any bad files without any valid inputs.
        locations_data.append(loc_data)

def export_as_json(locations_data : [[dict]]) -> None:
    # print(loc_data)
    json_file = json.dumps(locations_data)
    f = open("landmarks.json", "r")

    #TODO: generate a json name using loc_name parameter/ whatever name would be convenient
    with open("landmarks.json", "w") as outfile:
        outfile.write(json_file)

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
    filename = ""
    file = open("landmarks.json", "w") #Clears landmarks.json
    file.close()
    locations_data = []
    with open("file_pixel.csv", "r") as file:
        loc_data_txt = file.readlines()
        for line in loc_data_txt:
            name_data = line.split(",")
            filename = name_data[0] + "_" + name_data[1] + "_" + name_data[2] + ".txt"

            file_data = open_file(filename)
            pixel = (int(name_data[1]), int(name_data[2]))
            json_data = parse_data(file_data, pixel, name_data[0])
            consolidate(locations_data, json_data)
    print(locations_data)
    export_as_json(locations_data)





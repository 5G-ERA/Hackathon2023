import json
# import 5GEra client
from era_5g_client.client_base import NetAppClientBase, NetAppLocation
import math

# Open the file for reading
with open("laser_data.txt", "r") as fp:
    # Load the data from the file
    laser_data = json.load(fp)


def results(data):
    """ Callback which prints the results from the Network Application
    """
    print(data)

# Create the client library instance 
client = NetAppClientBase(results) 

# Register with the deployed network application and pass the arguments
client.register(NetAppLocation("localhost", 5896), args={"scan_width": 90})

# goes through the laser data and send them to the network application
for measurement in laser_data:
    r = [float(x) if not math.isinf(float(x)) else 9999 for x in measurement["ranges"]]
    client.send_json_http({"angle_min": float(measurement["angle_min"]), "angle_max": float(measurement["angle_max"]), "angle_increment": float(measurement["angle_increment"]), "ranges": r})
import json
# import 5GEra client
from era_5g_client.client import NetAppClient, MiddlewareInfo, FailedToConnect, RunTaskMode
import math
import os

# ip address or hostname of the middleware server
MIDDLEWARE_ADDRESS = os.getenv("MIDDLEWARE_ADDRESS", "127.0.0.1")
# middleware user
MIDDLEWARE_USER = os.getenv("MIDDLEWARE_USER", "00000000-0000-0000-0000-000000000000")
# middleware password
MIDDLEWARE_PASSWORD = os.getenv("MIDDLEWARE_PASSWORD", "password")
# middleware NetApp id (task id)
MIDDLEWARE_TASK_ID = os.getenv("MIDDLEWARE_TASK_ID", "00000000-0000-0000-0000-000000000000")
# middleware robot id (robot id)
MIDDLEWARE_ROBOT_ID = os.getenv("MIDDLEWARE_ROBOT_ID", "00000000-0000-0000-0000-000000000000")

# Open the file for reading
with open("laser_data.txt", "r") as fp:
    # Load the data from the file
    laser_data = json.load(fp)


def results(data):
    """ Callback which prints the results from the Network Application
    """
    print(data)
try:
    # creates an instance of NetApp client with results callback
    client = NetAppClient(results)
    # authenticates with the middleware
    client.connect_to_middleware(MiddlewareInfo(MIDDLEWARE_ADDRESS, MIDDLEWARE_USER, MIDDLEWARE_PASSWORD))
    # run task, wait until is ready and register with it
    client.run_task(MIDDLEWARE_TASK_ID, MIDDLEWARE_ROBOT_ID, True, RunTaskMode.WAIT_AND_REGISTER, args={"scan_width": 90})

    # goes through the laser data and send them to the network application
    for measurement in laser_data:
        r = [float(x) if not math.isinf(float(x)) else 9999 for x in measurement["ranges"]]
        client.send_json_http({"angle_min": float(measurement["angle_min"]), "angle_max": float(measurement["angle_max"]), "angle_increment": float(measurement["angle_increment"]), "ranges": r})
except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
except KeyboardInterrupt:
    print("Terminating...")
#except Exception as ex:
#    print(f"Failed to create client instance ({ex})")
finally:
    if client is not None:
        client.disconnect()

import roslibpy
# import scriptcontext as sc
import threading

# key = 'rosclient_%s' % str(ghenv.Component.InstanceGuid)
# previous_client = sc.sticky.get(key, None)

# if previous_client:
#     print('Disconnecting old client')
#     previous_client.terminate()

ros = roslibpy.Ros('127.0.0.1', 9090)
ros.run()
# sc.sticky[key] = ros

print('Connected=' + str(ros.is_connected))


wait_event = threading.Event()
topic = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')
collected_data = []
def handle_data(msg):
    collected_data.append(msg)
    if len(collected_data) >= 10:
        wait_event.set()

topic.subscribe(handle_data)

if not wait_event.wait(10):
    topic.unsubscribe()
    raise Exception('Nothing received')

print(collected_data)
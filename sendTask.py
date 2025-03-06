import socket
import EpuC2_pb2
import struct
from EpuC2_pb2 import Task, TaskType, Message, VehicleState, Status, LLA

MCAST_GRP = "224.0.254.254"
MCAST_PORT = 45654


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", MCAST_PORT))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)


def send_arm_command():
    
    vState = Status ( 
        vehicle_state = VehicleState(
        arm_state = 2
    ))

    task = Task(
        creator = "gcs",
        applies_to=["jb_1",  "jb_2","jb_3"], 
        task_type=TaskType.TASK_LAUNCH,  
    )

    lla = LLA(
        lat_d=37.7749,   # Latitude (example: 37.7749 for San Francisco)
        lon_d=-122.4194, # Longitude (example: -122.4194 for San Francisco)
        alt_m=100.0      # Altitude in meters (example: 100 meters)
    )




    # Create Message and attach task
    message_to_send = Message()
    message_to_send.task.append(task)
    message_to_send.status.append(vState)


    # Serialize the message to binary
    serialized_message = message_to_send.SerializeToString()

    # Send message to the multicast group
    sock.sendto(serialized_message, (MCAST_GRP, MCAST_PORT))
    print(f"ARM command sent to {MCAST_GRP}:{MCAST_PORT}")

send_arm_command()
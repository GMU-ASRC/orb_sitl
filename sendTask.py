import socket
import EpuC2_pb2
import struct
from EpuC2_pb2 import (
    Task, TaskType, Message, LLA, Goto_Task_Params, ControlPoint, ControlPointType,
    Set_GeoFence_Task_Params, ControlPointAction
)

MCAST_GRP = "224.0.254.254"
MCAST_PORT = 45654


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", MCAST_PORT))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)


def send_arm_command():
    
    # vState = Status ( 
    #     vehicle_state = VehicleState(
    #     arm_state = 2
    # ))

    # move = Status(
    #     creator = "gcs",
    #     applies_to=["*"], 
    #     task_type=TaskType.task_lla,  
    # )
    lla_geos = [
        LLA(
            lat_d = -89.9, 
            lon_d= -100,
            
        ),
        LLA(
            lat_d = 89.9, 
            lon_d= 100,
           
        ),
        LLA(
            lat_d = -50, 
            lon_d= -50,
           
        )
    ]

    poly_fence = Set_GeoFence_Task_Params(
        polygon_fence = lla_geos
    )

    
    task_geofence = Task(
        creator = "gcs",
        applies_to = ["*"], 
        task_type = TaskType.TASK_SET_GEOFENCE,
        set_geo_fence_task_params = poly_fence
    )
    lla = LLA(
        lat_d = 40,   # Latitude (example: 37.7749 for San Francisco)
        lon_d = -75, # Longitude (example: -122.4194 for San Francisco)
        alt_m = 10,      # Altitude in meters (example: 100 meters)
    )

    sphere_lla = LLA(
        lat_d = 36.12117784405506,
        lon_d = -115.16201374008685,
        alt_m = 100
    )


    controlPoint = ControlPoint(
        ctrl_pt_type = ControlPointType.GEODETIC,
        pos_lla = sphere_lla,
        action = ControlPointAction.DO_POS_HOLD
    )
    goto = Goto_Task_Params(
        tgt_pos = controlPoint,
    )
    task_launch = Task(
        creator = "gcs",
        applies_to = ["*"],
        task_type = TaskType.TASK_LAUNCH,
    )

    task_goto = Task(
        creator = "gcs",
        applies_to = ["*"], 
        task_type = TaskType.TASK_GOTO,
        goto_task_params = goto,
    )

   
    # task_pos_hold    
    # Create Message and attach task
    message_to_send = Message()
    # message_to_send.task.append(task_geofence)
    # message_to_send.task.append(task_launch)
    message_to_send.task.append(task_goto)

    # message_to_send.status.append(vState)
    # message_to_send.status(move)

    # Serialize the message to binary
    serialized_message = message_to_send.SerializeToString()

    # Send message to the multicast group
    sock.sendto(serialized_message, (MCAST_GRP, MCAST_PORT))
    print(f"ARM command sent to {MCAST_GRP}:{MCAST_PORT}")

send_arm_command()
import socket
import EpuC2_pb2


MCAST_GRP = "224.0.254.254"
MCAST_PORT = 45654


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", MCAST_PORT))


def send_arm(agent_id):

    task = EpuC2_pb2.Task()
    task.creator = "jb_1"
    task.applies_to = "jb_all"
    task.ArmState = 2
    task.TASK_LAUNCH = 7


    message = EpuC2_pb2.Message()
    message.task_append()

  
    return agent_id

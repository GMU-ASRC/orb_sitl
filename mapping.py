import socket
import struct
import EpuC2_pb2
import matplotlib.pyplot as plt


MCAST_GRP = "224.0.254.254"
MCAST_PORT = 45654


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", MCAST_PORT))


mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print(f"Listening for Protobuf UDP messages on {MCAST_GRP}:{MCAST_PORT}...")

plt.ion()
fig, ax = plt.subplots()

while True:
    data, addr = sock.recvfrom(4096) 

    message = EpuC2_pb2.Message()
    message.ParseFromString(data)

        
    for status in message.status:
            if status.HasField("nav_state"):
                nav = status.nav_state

                lat_d = status.nav_state.pos_lla.lat_d
                lon_d = status.nav_state.pos_lla.lon_d
                alt_m = status.nav_state.pos_lla.alt_m

                ax.cla()
                ax.scatter(lon_d, lat_d, c='red', marker='o', label="Position")

                
                ax.set_xlabel("Longitude")
                ax.set_ylabel("Latitude")
                ax.set_title("Test")

                ax.grid(True)
                ax.legend()
        


                plt.draw()
                plt.pause(.001)



  
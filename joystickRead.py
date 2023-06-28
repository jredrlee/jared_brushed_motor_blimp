import pygame
import time
import socket
import struct
import math

# udp params
UDP_IP = "192.168.0.79"#"192.168.0.79"
UDP_PORT = 1333
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    return sock

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick

def init():
    sock = udp_init()
    joystick = joystick_init()
    return sock, joystick

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))


if __name__ == "__main__":
    sock, joystick = init()
    l = 0.2 # meters
    
    while True:
        time_start = time.time()
        # Get the joystick readings
        pygame.event.pump()
        fx = -joystick.get_axis(1) # left handler: up-down, inverted
        taux = joystick.get_axis(0) # left handler: left-right
        fz = -joystick.get_axis(4)  # right handler: up-down, inverted
        tauz = joystick.get_axis(3) # right handler: left-right
        #print(fx, taux, fz, tauz)

        f1x = (fx - tauz/l)/2
        f2x = (fx + tauz/l)/2
        f1z = (fz + taux/l)/2
        f2z = (fz - taux/l)/2

        f1 = math.sqrt(f1x**2 + f1z**2) * 255/9
        f2 = math.sqrt(f2x**2 + f2z**2) * 255/9
        t1 = math.atan2(f1z, f1x)*180/math.pi
        t2 = math.atan2(f2z, f2x)*180/math.pi
        print(f1, f2, t1, t2) #f2
        # print()
        
        message = struct.pack('<ffff', fx *1000, fz * 1000, t1, t2) 
        udp_send(sock, UDP_IP, UDP_PORT, message)
        #print(message)
        while(time.time() - time_start < 0.01):
            # print("hello")
            time.sleep(0.002)

"""
Contents of this file are copyright Andrew Pullin, 2013
"""

#Base station
BS_COMPORT = 'COM17'
BS_BAUDRATE = 230400
#XBee
#BS_COMPORT = 'COM4'
#BS_BAUDRATE = 111111

deg2count = 14.375
count2deg = 1/deg2count

#For download timeout
last_packet_time = 0
pkts = 0

ROBOTS = []
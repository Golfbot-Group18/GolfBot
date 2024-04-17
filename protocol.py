import ev3_dc as ev3
with ev3.EV3(protocol=ev3.BLUETOOTH, host='74:DA:38:6B:13:0E') as my_robot:
    print(my_robot)

#A0:E6:F8:DA:EA:AF
#ev3dev
from auv_move import HaloAUV

auv = HaloAUV()
# auv.restart_pixhawk()
auv.arm()
auv.set_relative_depth(250.35)
auv.hold_depth()
# while(1):
#     auv.ascend(600)
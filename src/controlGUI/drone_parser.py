#
#     print_statement("Rangefinder: %s" % vhcl.rangefinder)
#     print_statement("Rangefinder distance: %s" % vhcl.rangefinder.distance)
#     print_statement("Rangefinder voltage: %s" % vhcl.rangefinder.voltage)
#     print_statement("Heading: %s" % vhcl.heading)
#     print_statement("Is Armable?: %s" % vhcl.is_armable)
#     print_statement("System status: %s" % vhcl.system_status.state)
#     print_statement("Mode: %s" % vhcl.mode.name)  # settable
#     print_statement("Armed: %s" % vhcl.armed)  # settable
#
#
# def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
#     """
#     Move vehicle in direction based on specified velocity vectors.
#     """
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#         0,       # time_boot_ms (not used)
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
#         0b0000111111000111, # type_mask (only speeds enabled)
#         0, 0, 0, # x, y, z positions (not used)
#         velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
#         0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#         0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
#
#
#     # send command to vehicle on 1 Hz cycle
#     for x in range(0,duration):
#         vehicle.send_mavlink(msg)
#         time.sleep(1)
#
# ######3
# # def goto(dNorth, dEast, vhcl, gotoFunction=land):
# #     currentLocation = vhcl.location.global_relative_frame
# #     targetLocation = get_location_metres(currentLocation, dNorth, dEast)
# #     targetDistance = get_distance_metres(currentLocation, targetLocation)
# #     gotoFunction(targetLocation)
# #
# #     while (vehicle.mode.name == "GUIDED") and (
# #             get_distance_metres(vehicle.home_location, vehicle.location.global_frame) < radius) and (
# #             vehicle.location.global_relative_frame.alt < alt_limit):
# #         # Stop action if we are no longer in guided mode or outside radius.
# #         remainingDistance = get_distance_metres(vehicle.location.global_frame, targetLocation)
# #
# #         print("Distance to target: ", remainingDistance)
# #         if remainingDistance <= targetDistance * 0.1:  # Just below target, in case of undershoot.
# #             print("Reached target")
# #             break
#
#
# conn_str = '/dev/ttyACM0'
# vehicle = initialize_drone(conn_str)
#
# print('change the drone mode to loiter')
# change_mode("GUIDED")
#
# arm(vehicle)
#
# print(f'state: {vehicle.system_status.state}')
# time.sleep(5)
#
# vehicle.airspeed = 1  # setting the airspeed to be slow
#
# # print_data(vehicle)
#
#
# arm_and_takeoff(vehicle, 3)
#
# print('reached altitude')
#
# time.sleep(5)
#
#
# print('moving the drone')
# send_ned_velocity(.5, .5, -.5, 3)
#
# time.sleep(5)
#
# print('moving the drone back')
# send_ned_velocity(-.5, -.5, .5, 3)
#
# time.sleep(5)
#
# # print('going back')
#
# print('now going to land')
#
# vehicle.mode = VehicleMode("LAND")
#
# print('now we wait till we reach the ground')
# time.sleep(20)
#
# # print('disarming the drone')
# disarm(vehicle)
#
# # print(vehicle.system_status.state) # good thing to get on the gui
# # print(vehicle.system_status.state)
#
# vehicle.close()

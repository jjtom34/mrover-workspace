import lcm
import sys
import time as t
import odrive as odv
import threading
# from ODriver_Req_State import *
# from ODriver_Pub_Encoders import *
# from ODriver_Req_Vel import *
# from ODriver_Pub_State import *
from rover_msgs import ODriver_Req_State, ODriver_Req_Vel, \
    ODriver_Pub_State, ODriver_Pub_Encoders
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CTRL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, \
    AXIS_STATE_IDLE
#from . import modrive as Modrive


def main():
    global lcm_
    lcm_ = lcm.LCM()

    # insert lcm subscribe statements here

    # These have been mapped

    #lcm_.subscribe("/odriver_req_state", odriver_req_state_callback)

    #lcm_.subscribe("/odrive_req_vel", odriver_req_vel_callback)

    global modrive
    global legalAxis
    legalAxis = sys.argv[2]
    
    if (legalAxis != "LEFT" and legalAxis != "RIGHT" and legalAxis != "BOTH"):
        print("invalid odrive axis given")

    global msg
    global msg1
    msg = ODriver_Pub_Encoders()
    msg1 = ODriver_Pub_State()

    threading._start_new_thread(lcmThreaderMan, ())

    i = 0 
    while True:
        #lcm_.handle()
        nextState()
        print("successfully booted up")
        print(i)
        i += 1
        t.sleep(1)

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/odriver_req_state", odriver_req_state_callback)
    lcm_1.subscribe("/odrive_req_vel", odriver_req_vel_callback)
    print("lc threader set up")
    while True:
        lcm_1.handle()
        print("lcm handling")
        t.sleep(1)


states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING", "EXIT"]
# Program states possible - BOOT, DISARMED, ARMED, ERROR, CALIBRATING, EXIT
# 							1		 2	      3	    4		 5         6
currentState = "BOOT"  # starting state
requestedState = "DISARMED"  # starting requested state
odrive = None  # starting odrive


def publish_state_msg(msg, state_number):
    global currentState
    currentState = states[state_number - 1]
    msg1.state = state_number
    msg1.serialid = sys.argv[1]
    lcm_.publish("/odriver_pub_state", msg1.encode())  # is lcm_ global?
    return t.time()


def publish_encoder_helper(msg, axis):
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estvel = modrive.get_vel_estimate(axis)
    msg.serialid = sys.argv[1]
    if (axis == "RIGHT"):
        msg.axis = 'r'
    elif (axis == "LEFT"):
        msg.axis = 'l'
    lcm_.publish("/odriver_pub_encoders", msg.encode())


def publish_encoder_msg(msg):
    if (legalAxis == "BOTH"):
        publish_encoder_helper(msg, "LEFT")
        publish_encoder_helper(msg, "RIGHT")
        return t.time()
    elif (legalAxis == "RIGHT"):
        publish_encoder_helper(msg, "RIGHT")
        return t.time()
    elif (legalAxis == "LEFT"):
        publish_encoder_helper(msg, "RIGHT")
        return t.time()


def nextState():
    print("Moving to next state")
    # every time the state changes,
    # publish an odrive_state lcm message, with the new state
    global currentState
    global requestedState
    global odrive
    global encoderTime
    global modrive

    if (currentState == "BOOT"):
        print("current state is boot")
        # attempt to connect to odrive
        odrive = odv.find_any(serial_number=sys.argv[1])
        modrive = Modrive(odrive)  # arguments = odr
        encoderTime = t.time()
        # Block until odrive is connected
        # set current limit on odrive to 100
        modrive.set_current_lim(legalAxis, 100)
        # set controller's control mode to velocity control
        modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        # set currentState to DISARMED
        print("setting state to disarmed")
        publish_state_msg(msg1, 2) #2 = disarmed

    elif (currentState == "DISARMED"):
        print("current state is disarmed")
        # if 100 ms have passed since last time data was published
        #   publish an odrive_data lcm message with measured
        #   current and estimated velocity

        # Unsure if using correct timestamp
        if (t.time() - encoderTime > 0.1):
            encoderTime = t.time()
            publish_encoder_msg(msg)
        # unsure if this is right
        errors = modrive.check_errors(legalAxis)
        if errors:
            # sets state to error
            print("found errors")
            publish_state_msg(msg1, 4)
        elif requestedState == "ARMED":
            print("setting state to armed")
            modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            # sets state to armed
            publish_state_msg(msg1, 3)
        elif requestedState == "BOOT":
            print("setting state to boot")
            odrive.reboot()
            # sets state to boot
            publish_state_msg(msg1, 1)
        elif requestedState == "CALIBRATING":
            print("setting state to calibrating")
            modrive.requested_state(legalAxis,
                                    AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
            # sets state to calibrating
            publish_state_msg(msg1, 5)

    elif (currentState == "ARMED"):
        print("current state is armed")
        if (encoderTime - t.time() > 0.1):
            encoderTime = publish_encoder_msg(msg)
        errors = odv.dump_errors(odrive)
        if errors:
            print("found errors")
            # sets state to error
            publish_state_msg(msg1, 4)
        elif requestedState == "DISARMED":
            print("setting state to disarmed")
            modrive.set_control_mode(legalAxis, AXIS_STATE_IDLE)
        # sets state to disarmed
            publish_state_msg(msg1, 2)
        elif requestedState == "BOOT":
            print("setting state to boot")
            odrive.reboot()
        # sets state to boot
            publish_state_msg(msg1, 1)
        elif requestedState == "CALIBRATING":
            print("setting state to calibrating")
            modrive.requested_state(legalAxis,
                                    AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        # sets state to calibrating
            publish_state_msg(msg1, 5)
        elif (currentState == "ERROR"):
            print ("current state is erorr")
            if requestedState == "BOOT":
                print("setting state to boot")
                odrive.reboot()
                # sets current state to boot
                publish_state_msg(msg1, 1)
            elif requestedState == "CALIBRATING":
                print("setting state to calibrating")
                modrive.requested_state(legalAxis,
                                        AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
                # sets current state to calibrating
                publish_state_msg(msg1, 5)
    elif (currentState == "CALIBRATING"):
        print("current state is calibrating")
        # if odrive is done calibrating
        #   set current limit on odrive to 100
        #   set controller's control mode to velocity control
        #   set currentState to DISARMED
        # We don't know how to check if done calibrating
        # if odrive.

        # TODO: add in check for finish calibration(axis == idle)
        if legalAxis == "LEFT":
            if modrive.get_current_state("LEFT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        elif legalAxis == "RIGHT":
            if modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        elif legalAxis == "BOTH":
            if modrive.get_current_state("LEFT") == AXIS_STATE_IDLE \
                        and modrive.get_current_state(
                            "RIGHT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)

    # sets state to disarmed
        publish_state_msg(msg1, 2)

    print("I SURVIVED NEXT STATE !!!")


def odriver_req_state_callback(channel, msg):
    message = ODriver_Req_State.decode(msg)
    if message.serialid == sys.argv[1]:
        requestedState = states[message.requestState - 1]
    # TODO: check which axis are legal
    if requestedState == "EXIT":
        if legalAxis == "LEFT":
            if modrive.get_vel_estimate("LEFT") == 0 and \
                    modrive.get_current_state("LEFT") == AXIS_STATE_IDLE:
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()
        elif legalAxis == "RIGHT":
            if modrive.get_vel_estimate("RIGHT") == 0 and \
                    modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()
        elif legalAxis == "BOTH":
            if modrive.get_vel_estimate("LEFT") == 0 and \
                    modrive.get_current_state("LEFT") == AXIS_STATE_IDLE \
                    and modrive.get_vel_estimate("RIGHT") == 0 \
                    and modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()


def odriver_req_vel_callback(channel, msg):
    # if the program is in an ARMED state
    #   set the odrive's velocity to the float specified in the message
    # no state change
    message = ODriver_Req_Vel.decode(msg)
    if message.serialid == sys.argv[1]:
        if(currentState == "ARMED"):
            modrive.requested_state(legalAxis, AXIS_STATE_CLOSED_LOOP_CONTROL)
            modrive.set_vel(legalAxis, message.vel)


if __name__ == "__main__":
    main()

class Modrive:

    def __init__(self, odr):
        self.odrive = odr
        self.left_axis = self.odrive.axis0
        self.right_axis = self.odrive.axis1

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def set_current_lim(self, axis, lim):
        if (axis == "LEFT"):
            self.left_axis.motor.config.current_lim = lim
        elif (axis == "RIGHT"):
            self.right_axis.motor.config.current_lim = lim
        else:
            self.left_axis.motor.config.current_lim = lim
            self.right_axis.motor.config.current_lim = lim

    # odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def set_control_mode(self, axis, mode):
        if (axis == "LEFT"):
            self.left_axis.controller.config.control_mode = mode
        elif (axis == "RIGHT"):
            self.right_axis.controller.config.control_mode = mode
        elif (axis == "BOTH"):
            self.left_axis.controller.config.control_mode = mode
            self.right_axis.controller.config.control_mode = mode

    # odrive.axis0.motor.current_control.Iq_measured
    def get_iq_measured(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.motor.current_control.Iq_measured
        elif(axis == "RIGHT"):
            return self.right_axis.motor.current_control.Iq_measured
        else:
            print("ERROR: cant get the measured iq for both motors at once")
            return 0

    # odrive.axis0.encoder.vel_estimate
    def get_vel_estimate(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.encoder.vel_estimate
        elif(axis == "RIGHT"):
            return self.right_axis.encoder.vel_estimate
        else:
            print("ERROR: cant get the velocity estimate for both motors at \
                    once")
            return 0

    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    # odrive.axis0.requested_state = AXIS_STATE_IDLE
    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def requested_state(self, axis, state):
        if (axis == "LEFT"):
            self.left_axis.requested_state = state
        elif (axis == "RIGHT"):
                self.right_axis.requested_state = state
        else:
            self.right_axis.requested_state = state
            self.left_axis.requested_state = state

    # odrive.axis0.encoder.vel_estimate == 0

    def est_vel(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.encoder.vel_estimate
        elif (axis == "RIGHT"):
            return self.right_axis.encoder.vel_estimate

    def set_vel(self, axis, vel):
        if (axis == "LEFT"):
            self.left_axis.controller.vel_setpoint = vel
        elif axis == "RIGHT":
            self.right_axis.controller.vel_setpoint = vel
        elif axis == "BOTH":
            self.left_axis.controller.vel_setpoint = vel
            self.right_axis.controller.vel_setpoint = vel
        else:
            print("ERROR, unknown axis")

    def get_current_state(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.current_state
        elif(axis == "RIGHT"):
            return self.right_axis.current_state
        else:
            print("cant get current state of both axes at once")
            return 0

    def check_errors(self, axis):
        left = self.left_axis.error
        right = self.right_axis.error
        if (axis == "LEFT"):
            return left
        elif axis == "RIGHT":
            return right
        else:
            return right+left

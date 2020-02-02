import lcm
import sys
import time as t
import odrive as odv
import threading
from rover_msgs import DriveStateCmd, DriveVelCmd, \
    DriveStateData, DriveVelData
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CTRL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, \
    AXIS_STATE_IDLE, ENCODER_MODE_HALL


def main():
    global lcm_
    lcm_ = lcm.LCM()

    global modrive
    global legalController
    legalController = int(sys.argv[1])
    global legalAxis
    legalAxis = sys.argv[2]
    if (legalAxis != "FRONT" and legalAxis != "BACK" and legalAxis != "BOTH"):
        print("invalid odrive axis given")

    global msg
    global msg1
    msg = DriveVelData()
    msg1 = DriveStateData()

    global lock
    lock = threading.Lock()

    global currentState
    currentState = "NONE"  # starting state
    global requestedState
    requestedState = "BOOT"  # starting requested state

    global encoderTime
    encoderTime = 0
    threading._start_new_thread(lcmThreaderMan, ())

    while True:
        # lcm_.handle()
        print(currentState)
        currentState = nextState(currentState)
        t.sleep(1)

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/drivestatecmd",
                    drivestatecmd_callback)
    lcm_1.subscribe("/drivevelcmd", drivevelcmd_callback)
    while True:
        lcm_1.handle()
        t.sleep(1)


states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING", "EXIT", "NONE"]
# Program states possible - BOOT,  DISARMED, ARMED, ERROR, CALIBRATING, EXIT, NONE
# 							1		 2	      3	      4       5          6      7


odrive = None  # starting odrive


def publish_state_msg(msg, state_number):
    msg.state = state_number
    msg.controller = int(sys.argv[1])
    lcm_.publish("/drivestatedata", msg.encode())
    print("changed state to " + states[state_number - 1])
    return states[state_number - 1]


def publish_encoder_helper(msg, axis):
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estimatedVel = modrive.get_vel_estimate(axis)

    if (axis == "BACK"):
        if (legalController == 0):
            msg.axis = 2
        if (legalController == 1):
            msg.axis = 3
    elif (axis == "FRONT"):
        if (legalController == 0):
            msg.axis = 0
        if (legalController == 1):
            msg.axis = 1
    lcm_.publish("/driveveldata", msg.encode())


def publish_encoder_msg(msg):
    if (legalAxis == "BOTH"):
        publish_encoder_helper(msg, "FRONT")
        publish_encoder_helper(msg, "BACK")
        return t.time()
    elif (legalAxis == "BACK"):
        publish_encoder_helper(msg, "BACK")
        return t.time()
    elif (legalAxis == "FRONT"):
        publish_encoder_helper(msg, "FRONT")
        return t.time()


def nextState(currentState):
    lock.acquire()
    # every time the state changes,
    # publish an odrive_state lcm message, with the new state
    # global currentState
    global requestedState
    global odrive
    global encoderTime
    global modrive
    print("Requested State: " + requestedState)
    print("Current State: " + currentState)

    if (currentState != requestedState):
        currentState = change_state(currentState)

    try:
        if (currentState == "BOOT"):
            # attempt to connect to odrive
            print("looking for odrive")

            if sys.argv[1] == "0":
                id = "2091358E524B"
            if sys.argv[1] == "1":
                id = "20563591524B"
            print(id)
            odrive = odv.find_any(serial_number=id)
            t.sleep(3)
            print("found odrive")
            modrive = Modrive(odrive)  # arguments = odr
            encoderTime = t.time()
            #reset odrive 
            if(legalAxis != "BOTH"):
                modrive.reset(legalAxis)
            else:
                modrive.reset("FRONT")
                modrive.reset("BACK")
            # Block until odrive is connected
            # set current limit on odrive to 100
            modrive.set_current_lim(legalAxis, 100)
            # set controller's control mode to velocity control
            modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            # set currentState to DISARMED
            print(currentState)
            requestedState = publish_state_msg(msg1, 2)
        elif (currentState == "DISARMED"):
            if (t.time() - encoderTime > 0.1):
                print("Sent Encoder Message")
                encoderTime = t.time()
                publish_encoder_msg(msg)
            modrive.requested_state(legalAxis, AXIS_STATE_CLOSED_LOOP_CONTROL)
            # Calibration sets this to idle we need thi to set vel to 0
            modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            if legalAxis == "FRONT":
                if modrive.get_vel_estimate("FRONT") != 0:
                    modrive.set_vel(legalAxis, 0)
            elif legalAxis == "BACK":
                    if modrive.get_vel_estimate("BACK") != 0:
                        modrive.set_vel(legalAxis, 0)
            elif legalAxis == "BOTH":
                    if (modrive.get_vel_estimate("FRONT") != 0 and
                            modrive.get_vel_estimate("BACK") != 0):
                            modrive.set_vel(legalAxis, 0)
            modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
            errors = modrive.check_errors(legalAxis)
            if errors:
                # sets state to error
                requestedState = publish_state_msg(msg1, 4)

        elif (currentState == "ARMED"):
            if (encoderTime - t.time() > 0.1):
                print("Sent Encoder Message")
                encoderTime = publish_encoder_msg(msg)
            errors = modrive.check_errors(legalAxis)

            if errors:
                # sets state to error
                requestedState = publish_state_msg(msg1, 4)

        elif (currentState == "ERROR"):
            # sets current state to calibrating
            requestedState = publish_state_msg(msg1, 1)
        elif (currentState == "CALIBRATING"):
            # if odrive is done calibrating
            #   set current limit on odrive to 100
            #   set controller's control mode to velocity control
            #   set currentState to DISARMED
            # We don't know how to check if done calibrating

            # TODO: add in check for finish calibration(axis == idle)
            if legalAxis == "FRONT":
                if modrive.get_current_state("FRONT") == AXIS_STATE_IDLE:
                    modrive.set_current_lim(legalAxis, 100)
                    modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            elif legalAxis == "BACK":
                if modrive.get_current_state("BACK") == AXIS_STATE_IDLE:
                    modrive.set_current_lim(legalAxis, 100)
                    modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            elif legalAxis == "BOTH":
                if modrive.get_current_state("FRONT") == AXIS_STATE_IDLE \
                            and modrive.get_current_state(
                                "BACK") == AXIS_STATE_IDLE:
                    modrive.set_current_lim(legalAxis, 100)
                    modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)

        # sets state to disarmed
            requestedState = publish_state_msg(msg1, 2)
        
        lock.release()
    except AttributeError:
        print("odrive disconnected")
        requestedState = "NONE"

    return currentState


def change_state(currentState):
    global requestedState
    try:
        if(requestedState == "BOOT"):  # happens when you request boot
            if (currentState == "ARMED" or currentState == "DISARMED"
                    or currentState == "ERROR" or currentState == "NONE"):
                if(currentState != "NONE"):
                    # Doesn't run on start b/c no current state
                    requestedState = "DISARMED"  # makes sure it goes to find it
                    try:
                        print('rebooting')
                        odrive.reboot()  # only runs after initial pairing
                    except:
                        print('channel error caught')

                currentState = publish_state_msg(msg1, 1)  # boot
            else:
                requestedState = currentState  # could not change state

        elif(requestedState == "DISARMED"):
            if (currentState == "NONE"):
                currentState = publish_state_msg(msg1, 1) #set it to boot, 1 = boot
            else:     
                currentState = publish_state_msg(msg1, 2)  # 2 = disarmed

        elif(requestedState == "ARMED"):
            if (currentState == "DISARMED"):
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
                currentState = publish_state_msg(msg1, 3)
                # sets current state to armed
            else:
                requestedState = currentState  # couldn't change state

        elif(requestedState == "CALIBRATING"):
            if (currentState == "DISARMED" or currentState == "ARMED" or
                    currentState == "BOOT"):
                if legalAxis != "BOTH":
                    modrive.requested_state(legalAxis,
                                        AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
                    while (modrive.get_current_state(legalAxis) != AXIS_STATE_IDLE):
                        t.sleep(0.1)

                elif legalAxis == "BOTH":
                    modrive.requested_state("FRONT",
                                        AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
                    while (modrive.get_current_state("FRONT") != AXIS_STATE_IDLE):
                        t.sleep(0.1)
                    modrive.requested_state("BACK",
                                        AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
                    while(modrive.get_current_state("BACK") != AXIS_STATE_IDLE):
                        t.sleep(0.1)
                currentState = publish_state_msg(msg1, 5)
            else:
                requestedState = currentState  # was not able to change states

        elif(requestedState == "ERROR"):  # Never go here
            currentState = publish_state_msg(msg1, 4)

        elif(requestedState == "NONE"):
            currentState = publish_state_msg(msg1, 7)

    except AttributeError:
        print("odrive disconnected")
        currentState = publish_state_msg(msg1, 7)
    return currentState


def drivestatecmd_callback(channel, msg):
    print("requested state call back is being called")
    global requestedState
    global modrive
    lock.acquire()
    message = DriveStateCmd.decode(msg)
    if message.controller == int(sys.argv[1]):  # Check which controller
        if (message.state == 1):
            requestedState = "DISARMED"
        elif (message.state == 2):
            requestedState = "ARMED"
        elif (message.state == 3):
            requestedState = "CALIBRATING"

    # if requestedState == "EXIT":
    #     if legalAxis == "FRONT":
    #         if modrive.get_vel_estimate("FRONT") == 0 and \
    #                 modrive.get_current_state("FRONT") == AXIS_STATE_IDLE:
    #             sys.exit()
    #         else:
    #             modrive.set_vel(legalAxis, 0)
    #             modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
    #             sys.exit()
    #     elif legalAxis == "BACK":
    #         if modrive.get_vel_estimate("BACK") == 0 and \
    #                 modrive.get_current_state("BACK") == AXIS_STATE_IDLE:
    #             sys.exit()
    #         else:
    #             modrive.set_vel(legalAxis, 0)
    #             modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
    #             sys.exit()
    #     elif legalAxis == "BOTH":
    #         if modrive.get_vel_estimate("FRONT") == 0 and \
    #                 modrive.get_current_state("FRONT") == AXIS_STATE_IDLE \
    #                 and modrive.get_vel_estimate("BACK") == 0 \
    #                 and modrive.get_current_state("BACK") == AXIS_STATE_IDLE:
    #             sys.exit()
    #         else:
    #             modrive.set_vel(legalAxis, 0)
    #             modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
    #             sys.exit()
    lock.release()


def drivevelcmd_callback(channel, msg):
    # if the program is in an ARMED state
    #   set the odrive's velocity to the float specified in the message
    # no state change
    lock.acquire()
    global currentState
    global modrive
    global legalAxis
    message = DriveVelCmd.decode(msg)
    try:
        if(currentState == "ARMED"):
            if (legalController == 0):
                if (message.axis == 0):
                        modrive.requested_state(legalAxis,
                                                AXIS_STATE_CLOSED_LOOP_CONTROL)
                        modrive.set_vel("FRONT", message.vel)
                if (message.axis == 2):
                        modrive.requested_state(legalAxis,
                                                AXIS_STATE_CLOSED_LOOP_CONTROL)
                        modrive.set_vel("BACK", message.vel)
            if (legalController == 1):
                if (message.axis == 1):
                        modrive.requested_state(legalAxis,
                                                AXIS_STATE_CLOSED_LOOP_CONTROL)
                        modrive.set_vel("FRONT", message.vel)
                if (message.axis == 3):
                        modrive.requested_state(legalAxis,
                                                AXIS_STATE_CLOSED_LOOP_CONTROL)
                        modrive.set_vel("BACK", message.vel)
    except AttributeError:
        print("odrive disconnected")

    lock.release()


if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 30

    def __init__(self, odr):
        self.odrive = odr
        self.front_axis = self.odrive.axis0
        self.back_axis = self.odrive.axis1

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def set_current_lim(self, axis, lim):
        if (lim > self.CURRENT_LIM):
            lim = self.CURRENT_LIM
        if (axis == "FRONT"):
            self.front_axis.motor.config.current_lim = lim
        elif (axis == "BACK"):
            self.back_axis.motor.config.current_lim = lim
        elif (axis == "BOTH"):
            self.front_axis.motor.config.current_lim = lim
            self.back_axis.motor.config.current_lim = lim

    # odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def set_control_mode(self, axis, mode):
        if (axis == "FRONT"):
            self.front_axis.controller.config.control_mode = mode
        elif (axis == "BACK"):
            self.back_axis.controller.config.control_mode = mode
        elif (axis == "BOTH"):
            self.front_axis.controller.config.control_mode = mode
            self.back_axis.controller.config.control_mode = mode

    # odrive.axis0.motor.current_control.Iq_measured
    def get_iq_measured(self, axis):
        if (axis == "FRONT"):
            return self.front_axis.motor.current_control.Iq_measured
        elif(axis == "BACK"):
            return self.back_axis.motor.current_control.Iq_measured
        else:
            print("ERROR: cant get the measured iq for both motors at once")
            return 0

    # odrive.axis0.encoder.vel_estimate
    def get_vel_estimate(self, axis):
        if (axis == "FRONT"):
            return self.front_axis.encoder.vel_estimate
        elif(axis == "BACK"):
            return self.back_axis.encoder.vel_estimate
        else:
            print("ERROR: cant get the velocity estimate for both motors at \
                    once")
            return 0

    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    # odrive.axis0.requested_state = AXIS_STATE_IDLE
    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def requested_state(self, axis, state):
        if (axis == "FRONT"):
            self.front_axis.requested_state = state
        elif (axis == "BACK"):
                self.back_axis.requested_state = state
        elif (axis == "BOTH"):
            self.back_axis.requested_state = state
            self.front_axis.requested_state = state

    # odrive.axis0.encoder.vel_estimate == 0

    def set_vel(self, axis, vel):
        if (axis == "FRONT"):
            self.front_axis.controller.vel_setpoint = vel
        elif axis == "BACK":
            self.back_axis.controller.vel_setpoint = vel
        elif axis == "BOTH":
            self.front_axis.controller.vel_setpoint = vel
            self.back_axis.controller.vel_setpoint = vel
        else:
            print("ERROR, unknown axis")

    def get_current_state(self, axis):
        if (axis == "FRONT"):
            return self.front_axis.current_state
        elif(axis == "BACK"):
            return self.back_axis.current_state
        else:
            print("cant get current state of both axes at once")
            return 0
    def reset(self, axis):
        m_axis = self.front_axis
        if (axis == "BACK"):
            m_axis = self.back_axis
        m_axis.motor.config.pole_pairs = 15
        m_axis.motor.config.resistance_calib_max_voltage = 4
        m_axis.motor.config.requested_current_range = 25 #Requires config save and reboot
        m_axis.motor.config.current_control_bandwidth = 100

        m_axis.encoder.config.mode = ENCODER_MODE_HALL
        m_axis.encoder.config.cpr = 90

        m_axis.encoder.config.bandwidth = 100
        m_axis.controller.config.pos_gain = 1
        m_axis.controller.config.vel_gain = 0.02
        m_axis.controller.config.vel_integrator_gain = 0.1
        m_axis.controller.config.vel_limit = 1000
        m_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def check_errors(self, axis):
        front = self.front_axis.error
        back = self.back_axis.error
        if (axis == "FRONT"):
            return front
        elif axis == "BACK":
            return back
        else:
            return back+front

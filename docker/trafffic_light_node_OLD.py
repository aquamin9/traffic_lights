#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8
from rgb_led import RGB_LED
from std_msgs.msg import String


# COLORS and FREQUENCIES of lights
# go_right : f4 and purple
# go_left  : f5 and yellow
# go_forward:f4 and green
# stop 	  : f3 and red
class TrafficLight(object):

    def __init__(self):
        # Hardcoded color values to configure traffic light
        # ATTENTION: This skript uses GRB instead of RGB logic to work with the
        # newest version of the traffic lights with surface mounted LEDs.
        self.green_color = [1, 0, 0]
        self.red_color = [0, 1, 0]
        self.yellow_color = [1, 1, 0]
        self.black_color = [0, 0, 0]

        self.blue_color = [0, 0, 1]
        # GRB logic
        self.purple_color = [0, 1, 1]
        self.cyan_color = [1, 0, 1]

        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.cycle = None

        # Getting the list of reference colors and frequencies
        self.protocol = self.setupParameter("~LED_protocol", [])  # should be a list of tuples

        # FREQUENCIES
        # Get red and green light frequency
        self.stop_light_freq = self.protocol['signals']['traffic_light_stop']['frequency']
        self.go_right_freq = self.protocol['signals']['CAR_SIGNAL_B']['frequency']
        self.go_forward_freq = self.protocol['signals']['traffic_light_go']['frequency']
        self.go_left_freq = self.protocol['signals']['CAR_SIGNAL_C']['frequency']
        # Get the order of the lights
        self.traffic_light_list = self.setupParameter("~traffic_light_list", [0, 2, 3, 4])

        # LIGHT DURATIONS
        # Get go_light duration in seconds
        self.go_light_duration = self.setupParameter("~go_light_duration", 5)
        # get stop_light duration in seconds
        self.stop_light_duration = self.setupParameter("~stop_light_duration", 4)

        # PERIODS
        self.stop_t = 1.0 / self.stop_light_freq

        self.go_forward_t = 1.0 / self.go_forward_freq
        self.go_right_t = 1.0 / self.go_right_freq
        self.go_left_t = 1.0 / self.go_left_freq

        # The particular light is initially red
        self.light_on = False
        
		# go_states: FORWARD, RIGHT, LEFT
        # go_states initialization
        self.go_state = ''
        # Initialize all the lights with red
        self.stoplightlist = [0,2,3,4]
        rospy.loginfo("@start stoplightlist :"+str(self.stoplightlist))

        # all LEDs are off
        self.traffic_light_state = {0: False, 2: False, 3: False, 4: False}
        # yellowlightlist contains all the indices of lights which should emit yellow light
        #self.yellowlightlist = []
        # Initialise the light_index and its number (not sure if it is necessary in this case)
        self.light_index = 2
        self.light = self.traffic_light_list[self.light_index] #light = 3
		# SUBSCRIBER
        # (instead of a string there must be a message type with an index and a string.Index will correspond to the self.light_index and the string will provide information about go_right, go_left or go_forward)
        self.sub_switch_light = rospy.Subscriber("~switch_light", String, self.cbSwitchLight)
        # CYCLES
        rospy.loginfo("cycles start")
        self.red_LED_cycle = rospy.Timer(rospy.Duration(0.5 * self.stop_t), self.freqred)
        self.goright_LED_cycle = rospy.Timer(rospy.Duration(0.5 * self.go_right_t), self.freq_go_right)
        self.goleft_LED_cycle = rospy.Timer(rospy.Duration(0.5 * self.go_left_t), self.freq_go_left)
        self.goforward_LED_cycle = rospy.Timer(rospy.Duration(0.5 * self.go_forward_t), self.freq_go_forward)
        rospy.loginfo("cycles end")

        




    def cbSwitchLight(self, msg):
        # Get go_state
        # TODO:define a message with the required specifications
        self.go_state = msg.data
        #self.yellowlightlist = []
        # TODO:define a message with the required specifications
        #self.light_index = msg.index
        self.light = self.traffic_light_list[self.light_index]
        self.light_on = True
        #self.stoplightlist = self.traffic_light_list[:self.light_index] + self.traffic_light_list[(self.light_index + 1):]
        self.stoplightlist =[0,2,4]
        rospy.loginfo("sleep start")
        rospy.sleep(self.go_light_duration)  # Keep the light on$
        rospy.loginfo("sleep end")
        self.light_on = False
        self.stoplightlist = [0,2,3,4]

        #self.yellowlightlist = [self.light]
        #rospy.sleep(self.stop_light_duration)
        rospy.loginfo("cbSwitchLight callback finished")




    def freqred(self, event):
        rospy.loginfo("stoplightlist:"+str(self.stoplightlist))
        rospy.loginfo("traffic_light_state:"+ str(self.traffic_light_state))
        for l in self.stoplightlist:
            rospy.loginfo("Iterator:"+str(l))
            if self.traffic_light_state[l] == True:
                self.led.setRGB(l, self.black_color)
                self.traffic_light_state[l] = False
            else:
                self.led.setRGB(l, self.red_color)
                rospy.loginfo("RED on")
                self.traffic_light_state[l] = True
        '''
        for l in self.yellowlightlist:
            if self.traffic_light_state[l] == True:
                self.led.setRGB(l, self.black_color)
                self.traffic_light_state[l] = False

            else:
                self.led.setRGB(l, self.yellow_color)
                self.traffic_light_state[l] = True
        '''


    def freq_go_right(self, event):
        if self.light_on == False or self.go_state != 'RIGHT':  # Exit if lights should all be red
            return


        if self.traffic_light_state[self.light] == True:
            self.led.setRGB(self.light, self.black_color)
            self.traffic_light_state[self.light] = False
        else:
            self.led.setRGB(self.light, self.purple_color)
            self.traffic_light_state[self.light] = True
        rospy.loginfo("freq_go_right callback finished")


    def freq_go_left(self, event):
        if self.light_on == False or self.go_state != 'LEFT':  # Exit if lights should all be red
            return


        if self.traffic_light_state[self.light] == True:
            self.led.setRGB(self.light, self.black_color)
            self.traffic_light_state[self.light] = False
        else:
            self.led.setRGB(self.light, self.blue_color)
            self.traffic_light_state[self.light] = True
        rospy.loginfo("freq_go_left callback finished")


    def freq_go_forward(self, event):
        if self.light_on == False or self.go_state != 'FORWARD':  # Exit if lights should all be red
            return


        if self.traffic_light_state[self.light] == True:
            self.led.setRGB(self.light, self.black_color)
            self.traffic_light_state[self.light] = False
        else:
            self.led.setRGB(self.light, self.green_color)
            self.traffic_light_state[self.light] = True
        rospy.loginfo("freq_go_forward callback finished")


    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)


        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('traffic_light', anonymous=False)
    node = TrafficLight()
    rospy.spin()

#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, Int32, Bool
from geometry_msgs.msg import Point
from duckietown_msgs.msg import AprilTagDetection
from rgb_led import RGB_LED
from std_msgs.msg import String
#for nested dictionary initialization
from collections import defaultdict
from duckietown_utils import tcp_communication 


# COLORS and FREQUENCIES of lights
#frequencies 
#f3 = 5.7
#f4=7.8
#f5=10.6
#f2=2.4
#f1=1.9
# purplelight : (CAR_SIGNAL_B):f2 and purple
# greenlight  : f4 and green
# redlight :  (traffic_light_stop) f3 and green
# yellowlight 	  : (CAR_SIGNAL_C):f5 and yellow


#TODO: 
#Define new signals in LED_protocol.yaml 
#redlight =^ traffic_light_stop
#greenlight =^ go_forward
#purplelight =^ go_left 
#yellowlight =^ go_right 
class TrafficLight(object):

    def __init__(self):
        #----------------------Initialization TL START----------------------#


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

        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.cycle = None
        
        self.protocol = self.setupParameter("~LED_protocol", []) 

        #FREQUENCIES 
        # f1 = 1.9 
        
        self.greenlight_freq = self.protocol['signals']['CAR_SIGNAL_A_OLD']['frequency']
        #f2 = 4
        self.purplelight_freq = self.protocol['signals']['CAR_SIGNAL_A']['frequency']
        #f3 = 5.7
        self.redlight_freq =self.protocol['signals']['CAR_SIGNAL_GREEN']['frequency']
        #f4 = 7.8
        self.yellowlight_freq = self.protocol['signals']['traffic_light_go']['frequency']

        #PERIODS
        self.redlight_t = 1.0/self.redlight_freq 
        self.greenlight_t = 1.0/self.greenlight_freq
        self.purplelight_t = 1.0/self.purplelight_freq
        self.yellowlight_t = 1.0/self.yellowlight_freq
        #GREEN TIME 
        self.green_time = 30 
        rospy.loginfo("Green Time is "+ str(self.green_time))


        #List of red lights
        self.redlight_list = [0,2,3,4]
        self.light_number_special = 0
        self.light_color_special = "red"
        
        

        #Light States
        self.light_state_dict = {0:False , 2:False, 3:False, 4:False}

        #Subscriber 
        self.sub_traffic_light_switch = rospy.Subscriber("~traffic_light_switch",String,self.cbTrafficLight_switch)
        self.sub_green_time = rospy.Subscriber("~green_time",Int8,self.cbGreenTime)
        self.timer_red = rospy.Timer(rospy.Duration(0.5*self.redlight_t),self.cbTimerRed)
        self.charging_1_switch = rospy.Subscriber("~charging_1_switch", Int8, self.cbcharging_1_switch)
        self.charging_2_switch = rospy.Subscriber("~charging_2_switch", Int8, self.cbcharging_2_switch)

        self.charger1_capacity = 2
        self.charger2_capacity = 2
        self.charger1_size = 0
        self.charger2_size = 0
        self.charger1_size_old = self.charger1_size
        self.charger2_size_old = self.charger2_size
        self.charger_next_free = 2
        self.charger1_full = False 
        self.charger2_full = False

        rospy.Timer(rospy.Duration(1.0),self.cbChargingManager) #starting timer for ChargingManager
        #----------------------Initialization TL END----------------------#

        #----------------------Initialization MAINTENANCE START----------------------#

        #Initialize a point 
        self.initial_point = Point()
        #Initialize the localization tags according to their directional meaning (input as strings)
        self.entrance = '196'
        self.exit = '61'
        self.direction_CH1 = '342'
        self.direction_CH2 = '343' 

        #Get the static April Tags (right now it is hardcoded)
        #keys : apriltag ID values: positions 
        #TODO: Find a way to get the tag ids automaticaly via a YAML file...etc
        self.staticAprilTags = {self.entrance:{'position':self.initial_point},\
        self.direction_CH1:{'position':self.initial_point},\
        self.direction_CH2:{'position':self.initial_point},\
        self.exit:{'position':self.initial_point}}
        
        #Save the moving April Tags as {'tagID':{'position':Point,'neighbor':tagIDNeighbor,'timestamp':time,'direction':directions}}
        #this dictionary will be updated periodicaly 
        self.movingAprilTags = defaultdict(dict)

        #TODO:change the lanch file accordingly to load the parameters 
        #path :catkin_ws/src/20-indefinite-navigation/apriltags_ros/signs_and_tags/apriltagsDB.yaml
        '''
        self.AprilTags = self.self.setupParameter("~apriltagsDB", [])
        self.VehicleTags =str( [obj['tag_id'] for obj in self.AprilTags if obj[tag_type] == 'Vehicle'])
        '''
        #TODO:WRITE THE AT OF DB ACCORDINGLY
        self.VehicleTags = [str(x) for x in range(400,500)]
        #self.LocalizationTags =str( [obj['tag_id'] for obj in self.AprilTags if obj[tag_type] == 'Localization'])



        #Update Time of charger sizes(in seconds)
        self.updateChargerSizeTime = 1.0

        
        
        #Time Threshold for finding out which position  was the last one
        self.threshold = 5.0
        #
        self.bookkeeping_time = 1.0
        #'duckiebot':timestamp_secs will be saved in dictionaries of chargers 
        self.chargers ={'charger1':{},'charger2':{}}
        self.sub_poses = rospy.Subscriber("/poses_acquisition/poses",AprilTagDetection,self.cbPoses)
        #self.sub_reset = rospy.Subscriber("~reset",Bool,self.cbReset)
        #self.resetted = False
        
        self.timer_updateChargerSizes=rospy.Timer(rospy.Duration(self.updateChargerSizeTime),self.updateChargerSizes)
        self.timer_last_neighbor = rospy.Timer(rospy.Duration(self.bookkeeping_time),self.updateLastNeighbor)
        self.debug = rospy.Timer(rospy.Duration(5),self.DebugLists)

        #----------------------Initialization MAINTENANCE END----------------------#
        '''
        #----------------------Initialization TCP START----------------------#
        self.sub_TCP = rospy.Subscriber("~tcp_data",Int8,self.cbTCP)
        #----------------------Initialization TCP END----------------------#
        '''
    '''
    #-----------------------------TCP START-----------------------------#
    def cbTCP(self,msg):
        data_to_write = msg.data 
        #reserve charger part
        charging_stations = tcp_communication.getVariable("charging_stations")
        rospy.loginfo("["+self.node_name+"] charging_stations list:"+str(charging_stations))
        success = tcp_communication.setVariable("charging_stations/station" + str(1) + "/free_spots", data_to_write)
        if(success):
            rospy.loginfo("["+self.node_name+"]Variable "+str(data_to_write)+" should be successfully set!")
            rospy.sleep(3.0)
            charging_stations = tcp_communication.getVariable("charging_stations")
            rospy.loginfo("["+self.node_name+"] charging_stations list:"+str(charging_stations))
        elif(sucess =="ERROR"):
            rospy.loginfo("["+self.node_name+"] Value was too long to fit insde the BUFFER_SIZE")
        else:
            rospy.loginfo("["+self.node_name+"]Variable "+str(data_to_write)+" could not be set!")

    #-----------------------------TCP STOP-----------------------------#  
    '''
    #-----------------------------LED Management START-----------------------------#  
    def cbGreenTime(self,msg):
        self.green_time = msg.data
        rospy.loginfo("Green Time is: "+str(self.green_time))

    def cbcharging_1_switch(self,msg):
        self.charger1_size =  msg.data

    def cbcharging_2_switch(self,msg):
        self.charger2_size = msg.data

    def lightToggle(self,light_number,light_color):
        
        #rospy.loginfo("lightToggle function started")

        if(self.light_state_dict[light_number] == True):
            self.led.setRGB(light_number, self.black_color)
            self.light_state_dict[light_number] = False
            #traffic light is switched off
        else:
            if(light_color == "red"):
                self.led.setRGB(light_number, self.red_color)
            elif(light_color == "green"):
                self.led.setRGB(light_number, self.red_color)
            elif(light_color == "purple"):
                self.led.setRGB(light_number, self.red_color)
            else:
                self.led.setRGB(light_number, self.red_color)

            self.light_state_dict[light_number] = True
            #traffic light is switched on 
        #rospy.loginfo("lightToggle function ended")

    def cbChargingManager(self, event):
        rospy.loginfo("["+self.node_name+"] charger_size 1:"+str(self.charger1_size)+" charger_size_2 "+str(self.charger2_size))

        if((self.charger1_size != self.charger1_size_old) or (self.charger2_size != self.charger2_size_old)): #only allowing check if charger_next_free was updated
            if(self.charger1_size > self.charger2_size): 
                self.charger_next_free = 2
                rospy.loginfo("[" + self.node_name + "]" + " Next free charger: " + str(self.charger_next_free))
                self.timer_red.shutdown()
                self.timer_red = rospy.Timer(rospy.Duration(0.5*self.redlight_t),self.cbTimerRed)
            else :
                self.charger_next_free = 1
                rospy.loginfo("[" + self.node_name + "]" + " Next free charger: " + str(self.charger_next_free))
                self.timer_red.shutdown()
                self.timer_red = rospy.Timer(rospy.Duration(0.5*self.greenlight_t),self.cbTimerRed)
            self.charger1_size_old = self.charger1_size
            self.charger2_size_old = self.charger2_size

        

    def cbTimerRed(self,event):
        for light_number in self.redlight_list:
            self.lightToggle(light_number,"red")

    def cbTimerSpecial(self,event):
        self.lightToggle(self.light_number_special,self.light_color_special)
    
    def cbTrafficLight_switch(self,msg):
        rospy.loginfo("cbTrafficlight start")

        #get message string
        #color is a string and number is an integer 
        color, number = self.getMSG(msg)
        self.light_number_special = number
        self.light_color_special = color

        rospy.loginfo("published: "+color+" "+str(number))

        #remove the light which should blink differently
        
        if(number in self.redlight_list): 
            self.redlight_list.remove(number)
        
        

        #kill the old timer 
        self.timer_red.shutdown()
        rospy.loginfo("timer_red killed")

        #start two new timers 
        #get color frequency and set timer  
        if(color == "green"):
            self.timer_special = rospy.Timer(rospy.Duration(0.5*self.greenlight_t),self.cbTimerSpecial)
        elif(color == "purple"):
            self.timer_special = rospy.Timer(rospy.Duration(0.5*self.purplelight_t),self.cbTimerSpecial)
        else:
            self.timer_special = rospy.Timer(rospy.Duration(0.5*self.yellowlight_t),self.cbTimerSpecial)
        
        #timer_special is the timer of the light which will blink differently than the other ones 
        self.timer_others = rospy.Timer(rospy.Duration(0.5*self.redlight_t),self.cbTimerRed)
        rospy.loginfo("timer_special and timer_others started")

        #wait for self.green_time seconds
        rospy.sleep(rospy.Duration(45))

        self.timer_special.shutdown()
        self.timer_others.shutdown()
        rospy.loginfo("timer_special and timer_others killed")

        #put the number back into the list and sort the list 
        self.redlight_list.append(number)
        self.redlight_list.sort()

        self. timer_red = rospy.Timer(rospy.Duration(0.5*self.redlight_t),self.cbTimerRed)
        rospy.loginfo("timer_red started")

        rospy.loginfo("cbTrafficlight end ")
        

    def getMSG(self,msg):#TODO: extract "" from the message string 
        #msg should be "#NUMBER#COLOR#"

        tmp = str(msg.data).split("#")
        number = int(tmp[1])
        color = tmp[2]
        return color, number 

    def cbDebug(self,msg):
        rospy.loginfo(str(msg))
        color, number = self.getMSG(msg)
        rospy.loginfo("color "+color+ "number : "+ str(number) )

 #-----------------------------LED Management END-----------------------------#   

 #-----------------------------Charger Management START-----------------------------# 
 #for debug purposes
    '''
    def cbReset(self,msg):
        if(msg.data == True):
            self.staticAprilTags = {self.entrance:{'position':self.initial_point},\
                self.direction_CH1:{'position':self.initial_point},\
                self.direction_CH2:{'position':self.initial_point}}
            self.movingAprilTags = defaultdict(dict)
            self.chargers ={'charger1':{},'charger2':{}}
    '''


    def DebugLists(self,event):
        rospy.loginfo("STATIC AT:"+ str(self.staticAprilTags))
        rospy.loginfo("MOVING AT:"+str(self.movingAprilTags))
        rospy.loginfo("CHARGERS "+str(self.chargers))


    def updateChargerSizes(self,event):
        for charger in self.chargers.keys():
            if(charger == 'charger1'):
                self.charger1_size = int(len(self.chargers[charger].keys()))
            elif(charger == 'charger2'):
                self.charger2_size = int(len(self.chargers[charger].keys()))
            else:
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in updateChargerSizes")

        if(self.charger1_size == self.charger1_capacity):
            self.charger1_full = True 
        elif(self.charger1_size < self.charger1_capacity):
            self.charger1_full = False 

        if(self.charger2_size == self.charger2_capacity):
            self.charger2_full =True
        elif(self.charger2_size < self.charger2_capacity):
            self.charger2_full =False
        #rospy.loginfo("["+self.node_name+"] Charger_1 full ? "+str(self.charger1_full)+" Charger_2 full ? "+str(self.charger2_full))


    #Delete the tagID from self.movingAprilTags dictionary
    def deleteBot(self,tagID):
        for botID in self.movingAprilTags.keys():
            if(tagID == botID):
                try:
                    del self.movingAprilTags[tagID]
                except KeyError:
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(self.movingAprilTags)+" movingAprilTags: "+str(self.movingAprilTags))
        

    def updateLastNeighbor(self,event):
        current_time = rospy.get_rostime().to_sec()


        for botID in self.movingAprilTags.keys():
            #If the last time stamp is not updated for a long time, take action according to the last neighbor
            rospy.loginfo(str(abs(current_time-self.movingAprilTags[botID]['timestamp']))+ " time difference "+ str(botID))
            if(abs(current_time-self.movingAprilTags[botID]['timestamp'])> self.threshold):
                if(self.movingAprilTags[botID]['neighbor'] == self.direction_CH1):
                    rospy.loginfo(str(botID)+" on its way to charger 1")
                    self.chargers['charger1'][botID] = self.movingAprilTags[botID]['timestamp']
                    self.deleteBot(botID) #deletes bot from self.movingAprilTags       
                elif(self.movingAprilTags[botID]['neighbor'] == self.direction_CH2):
                    rospy.loginfo(str(botID)+" on its way to charger 2")
                    self.chargers['charger2'][botID] = self.movingAprilTags[botID]['timestamp']
                    self.deleteBot(botID) #deletes bot from self.movingAprilTags
                #self.entrance      normally self.exit    
                elif((self.movingAprilTags[botID]['neighbor'] == self.entrance or self.movingAprilTags[botID]['neighbor'] == self.exit) and (botID in list(self.chargers['charger1'].keys()) or botID in list(self.chargers['charger2'].keys()))):
                    rospy.loginfo(str(botID)+" on its way to exit")
                    self.releasePlace(botID)
                    self.deleteBot(botID)
                #TODO:Test this part 
                elif self.movingAprilTags[botID]['neighbor'] == self.entrance or self.movingAprilTags[botID]['neighbor'] == self.exit  : 
                    rospy.loginfo("["+self.node_name+"] The last neighbor of "+str(botID)+" was the entrance tag "+str(self.entrance))
                    rospy.loginfo("["+self.node_name+"] Deleting "+str(botID)+" from self.movingtags")
                    self.deleteBot(botID)
                    if botID in list(self.chargers['charger1'].keys()) or botID in list(self.chargers['charger2'].keys()): 
                        rospy.loginfo("["+self.node_name+"] Deleting "+str(botID)+" from self.chargers")
                        self.releasePlace(botID)

                else : 
                    rospy.loginfo("["+self.node_name+"] Something unexpected has heppend in updateLastNeighbor")




    def releasePlace(self,tagID):
        #delete the tagID from self.chargers 

        for charger in self.chargers.keys():
            #List of bots in charger
            bots = list(self.chargers[charger].keys())

            #Delete the tagID from chargers dictionary
            if(tagID in bots):

                try:
                    del self.chargers[charger][tagID]
                    rospy.loginfo(str(tagID)+" released charger"+str(charger))
                except KeyError :
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(charger)+". Charger has "+str(bots))
    

    def isDuckieBot(self,tagID):
        #look up whether the april tag is owned by a duckiebot 
        if(tagID in self.VehicleTags):
            return True
        else:
            return False
    '''
    def isStaticTag(self,tagID):
        if(tagID in self.LocalizationTags):
            return True
        else:
            return False
    '''

    #Finds the neighbor april tag to the given moving april tag
    def NearestNeighbor(self,tagPose):
        nearestTag = ''
        nearestPosition = 16000000
        #{'tagID':{'position':Point,'neighbor':tagIDNeighbor,'timestamp':time,'charger':chargerID}}
        for tagIDNeighbor, Neighbor in self.staticAprilTags.items():
            #absolute value of distance between a static tag and amoving tag squared 
            d = (Neighbor['position'].x -tagPose.x)**2 + (Neighbor['position'].y -tagPose.y)**2 
            if(d < nearestPosition):
                nearestPosition = d
                nearestTag = tagIDNeighbor
        if(len(nearestTag) == 0):
            rospy.loginfo("["+self.node_name+"]No nearest neighbor can be found.")
        return nearestTag




    def cbPoses(self,msg):
        #rospy.loginfo("["+self.node_name+"]"+" cbPoses message arrived: TagId: "+str(msg.tag_id)+" center: "+str(msg.center))
        current_time = rospy.get_rostime().to_sec()

        tagID = str(msg.tag_id)
        #3D Point just using x and y to save the pixels of the center of an april tag
        tagPose = Point()
        tagPose.x = msg.center[0] 
        tagPose.y = msg.center[1]
        tagPose.z = 0

        

        if(tagID in self.staticAprilTags.keys()):
            self.staticAprilTags[tagID]['position'] = tagPose
             
        
        if(self.isDuckieBot(tagID)):
            self.movingAprilTags[tagID]['position'] = tagPose
            self.movingAprilTags[tagID]['timestamp'] = current_time
            self.movingAprilTags[tagID]['neighbor'] = self.NearestNeighbor(tagPose)

            #rospy.loginfo("["+self.node_name+"]"+"Duckiebot "+tagID+" is near to "+self.movingAprilTags[tagID]['neighbor'])

       
 #-----------------------------Charger Management END-----------------------------#   



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

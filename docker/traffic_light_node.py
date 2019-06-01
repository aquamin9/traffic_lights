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
        self.freq_CH1 = self.protocol['signals']['CAR_SIGNAL_A_OLD']['frequency']
        #f2 = 4
        self.freq_CH2 = self.protocol['signals']['CAR_SIGNAL_A']['frequency']
        #f3 = 5.7
        self.freq_CH3 =self.protocol['signals']['CAR_SIGNAL_GREEN']['frequency']
        #f4 = 7.8
        self.freq_CH4 = self.protocol['signals']['traffic_light_go']['frequency']

        #PERIODS
        self.T_CH1 = 1.0/self.freq_CH1
        self.T_CH2 = 1.0/self.freq_CH2
        self.T_CH3 = 1.0/self.freq_CH3 
        self.T_CH4 = 1.0/self.freq_CH4
        #GREEN TIME 
        self.green_time = 30 
        rospy.loginfo("Green Time is "+ str(self.green_time))


        #List of lights
        self.light_list = [0,2,3,4]

        
        

        #Light States
        self.light_state_dict = {0:False , 2:False, 3:False, 4:False}

        #Subscriber 
        self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)

        #CHARGER CAPACITIES
        self.charger1_capacity = 2
        self.charger2_capacity = 2
        self.charger3_capacity = 2
        self.charger4_capacity = 2

        #CHARGER SIZES
        self.charger1_size = 0
        self.charger2_size = 0
        self.charger3_size = 0
        self.charger4_size = 0

        #Previous CHARGER SIZES
        self.charger1_size_old = self.charger1_size
        self.charger2_size_old = self.charger2_size
        self.charger2_size_old = self.charger3_size
        self.charger2_size_old = self.charger4_size

        #Initializing the first frequency 
        self.charger_next_free = 2

        self.charger1_full = False 
        self.charger2_full = False
        self.charger3_full = False
        self.charger4_full = False



        rospy.Timer(rospy.Duration(1.0),self.cbChargingManager) #starting timer for ChargingManager
        
        #----------------------Initialization TL END----------------------#

        #----------------------Initialization MAINTENANCE START----------------------#

        #Initialize a point 
        self.initial_point = Point()
        #Initialize the localization tags according to their directional meaning (input as strings)
        #entrance and exit ATs must be on the same tile
        self.entrance = '196'
        self.exit = '61'
        #These must be on different tiles
        self.direction1 = '342'
        self.direction2 = '343' 

        #Get the static April Tags (right now it is hardcoded)
        #keys : apriltag ID values: positions 
        #TODO: Find a way to get the tag ids automaticaly via a YAML file...etc
        self.staticAprilTags = {self.entrance:{'position':self.initial_point},\
        self.direction1:{'position':self.initial_point},\
        self.direction2:{'position':self.initial_point},\
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
        self.chargers ={'charger1':{},'charger2':{},'charger3':{},'charger4':{}}
        
        #SUBSCRIBERS
        self.sub_poses = rospy.Subscriber("/poses_acquisition/poses",AprilTagDetection,self.cbPoses)
        
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

    def cbChargingManager(self, event):
        #rospy.loginfo("["+self.node_name+"] charger_size 1:"+str(self.charger1_size)+" charger_size_2 "+str(self.charger2_size))
        
        if((self.charger1_size != self.charger1_size_old) or (self.charger2_size != self.charger2_size_old) or (self.charger3_size != self.charger3_size_old) or (self.charger4_size != self.charger4_size_old)): #only allowing check if charger_next_free was updated
            #Find the next free charger 
            self.charger_next_free = self.findNextFreeCharger()
        
            #blink the LED according to the next free charger 
            if self.charger_next_free == 1 :
                self.timer_CH.shutdown()
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH1),self.cbTimerCH)
            elif self.charger_next_free == 2 :
                self.timer_CH.shutdown()
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)
            elif self.charger_next_free == 3 :
                self.timer_CH.shutdown()
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH3),self.cbTimerCH)
            elif self.charger_next_free == 4 :
                self.timer_CH.shutdown()
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH4),self.cbTimerCH)
            else : 
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in cbChargingManager")

            rospy.loginfo("["+self.node_name+"]The new next free charger is "+str(self.charger_next_free))
            #set the charger size memories to the current value
            self.charger1_size_old = self.charger1_size
            self.charger2_size_old = self.charger2_size
            self.charger2_size_old = self.charger3_size
            self.charger2_size_old = self.charger4_size 

    def findNextFreeCharger(self):
        #Find the most unoccupied charger. This statement corresponds finding the minimal self.charger_size
        min_charger_size = self.charger1_size
        #TL will send DBs to the chargers 1, 2, 3 and then 4 in order
        CH_sizes = [self.charger1_size,self.charger2_size,self.charger3_size,self.charger4_size]
        CH_full = [self.charger1_full,self.charger2_full, self.charger3_full,self.charger4_full]
        for i in range(0,len(CH_sizes)):
            if CH_sizes[i] < min_charger_size and not CH_full[i]:
                min_charger_size = CH_sizes[i] 
        
        if (self.charger1_full and self.charger2_full and self.charger3_full and self.charger4_full):
            rospy.loginfo("["+self.node_name+"] All chargers are full")
        

    def cbTimerCH(self,event):
        for light_number in self.light_list:
            self.lightToggle(light_number,"red")

    def lightToggle(self,light_number,light_color):
        
        #rospy.loginfo("lightToggle function started")

        if(self.light_state_dict[light_number] == True):
            self.led.setRGB(light_number, self.black_color)
            self.light_state_dict[light_number] = False
            #traffic light is switched off
        else:
            if(light_color == "red"):
                self.led.setRGB(light_number, self.red_color)
            else:
                self.led.setRGB(light_number, self.red_color)

            self.light_state_dict[light_number] = True
            #traffic light is switched on 
        #rospy.loginfo("lightToggle function ended")


 #-----------------------------LED Management END-----------------------------#   

 #-----------------------------Charger Management START-----------------------------# 


    def DebugLists(self,event):
        rospy.loginfo("###########################")
        rospy.loginfo("STATIC AT:"+ str(self.staticAprilTags))
        rospy.loginfo("MOVING AT:"+str(self.movingAprilTags))
        rospy.loginfo("CHARGERS "+str(self.chargers))
        rospy.loginfo("Charger1_size: "+str(self.charger1_size)+" charger1_full: "+str(self.charger1_full))
        rospy.loginfo("Charger2_size: "+str(self.charger2_size)+" charger2_full: "+str(self.charger2_full))
        rospy.loginfo("Charger3_size: "+str(self.charger3_size)+" charger3_full: "+str(self.charger3_full))
        rospy.loginfo("Charger4_size: "+str(self.charger4_size)+" charger4_full: "+str(self.charger4_full))
        rospy.loginfo("Next Free Charger: "+str(self.charger_next_free))


    def updateChargerSizes(self,event):
        for charger in self.chargers.keys():
            if(charger == 'charger1'):
                self.charger1_size = int(len(self.chargers[charger].keys()))

                if(self.charger1_size == self.charger1_capacity):
                    self.charger1_full = True
                elif( self.charger1_size < self.charger1_capacity):
                    self.charger1_full = False

            elif(charger == 'charger2'):
                self.charger2_size = int(len(self.chargers[charger].keys()))

                if(self.charger2_size == self.charger2_capacity):
                    self.charger2_full = True
                elif( self.charger2_size < self.charger2_capacity):
                    self.charger2_full = False

            elif(charger == 'charger3'):
                self.charger3_size = int(len(self.chargers[charger].keys()))

                if(self.charger3_size == self.charger3_capacity):
                    self.charger3_full = True
                elif( self.charger3_size < self.charger3_capacity):
                    self.charger3_full = False

            elif(charger == 'charger4'):
                self.charger4_size = int(len(self.chargers[charger].keys()))

                if(self.charger4_size == self.charger4_capacity):
                    self.charger4_full = True
                elif( self.charger4_size < self.charger4_capacity):
                    self.charger4_full = False


            else:
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in updateChargerSizes")




    #Delete the tagID from self.movingAprilTags dictionary
    def deleteMovingBot(self,tagID):
        for botID in self.movingAprilTags.keys():
            if(tagID == botID):
                try:
                    del self.movingAprilTags[tagID]
                    rospy.loginfo("["+self.node_name+"] "+str(botID)+" is deleted from self.movingAprilTags")
                except KeyError:
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(self.movingAprilTags)+" movingAprilTags: "+str(self.movingAprilTags))
        

    def updateLastNeighbor(self,event):
        current_time = rospy.get_rostime().to_sec()


        for botID in self.movingAprilTags.keys():
            #If the last time stamp is not updated for a long time, take action according to the last neighbor
            rospy.loginfo(str(abs(current_time-self.movingAprilTags[botID]['timestamp']))+ " time difference "+ str(botID))

            if(abs(current_time-self.movingAprilTags[botID]['timestamp'])> self.threshold):

                #FIRST:@entrance or @exit 
                #LAST:@direction1
                if (self.movingAprilTags[botID]['last_neighbor'] == self.direction1) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit):
                    rospy.loginfo(str(botID)+" on WAY 1")
                    #self.chargers['charger1'][botID] = self.movingAprilTags[botID]['timestamp']
                    #self.deleteMovingBot(botID) #deletes bot from self.movingAprilTags     

                #FIRST:@entrance or @exit 
                #LAST:@direction2
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.direction2) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit):
                    rospy.loginfo(str(botID)+" on WAY 2")
                    #self.chargers['charger2'][botID] = self.movingAprilTags[botID]['timestamp']
                    #self.deleteMovingBot(botID) #deletes bot from self.movingAprilTags

                #FIRST:@direction1 or @direction2
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.direction2 or self.movingAprilTags[botID]['first_neighbor'] == self.direction1) :
                    rospy.loginfo(str(botID)+" on WAY EXIT")
                    self releaseChargerSpot(botID)
                    self.deleteMovingBot(botID)
                
                #Some undesired states

                #FIRST:@entrance or @exit
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit) : 
                    rospy.loginfo("["+self.node_name+"] The Tag ID "+str(botID)+" is seen on the same tile at the first and last apperance. This should not be the case. Please check that the ground AprilTags are well positioned and the Duckiebot was not driving too fast")
                    self.deleteMovingBot(botID)

                #FIRST = LAST
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.movingAprilTags[botID]['first_neighbor']):
                    rospy.loginfo("[+"self.node_name+"] The Duckiebot with AT "+str(botID)+" may have stuck on way "+str(self.movingAprilTags[botID]['last_neighbor']))
                    rospy.loginfo("[+"self.node_name+"] Check whether the ground april tags are well positioned.")
                    self.deleteMovingBot(botID)
                    
                #FIRST:@direction1 or direction2
                #LAST:@direction1 or direction2
                elif(self.movingAprilTags[botID]['first_neighbor'] == self.direction1 or self.movingAprilTags[botID]['last_neighbor'] == self.direction2) and (self.movingAprilTags[botID]['first_neighbor'] == self.direction2 or self.movingAprilTags[botID]['last_neighbor'] == self.direction1))
                    rospy.loginfo("[+"self.node_name+"] Check whether the ground april tags are well positioned.")
                    rospy.loginfo("[+"self.node_name+"] There might be an issue with the intersection control in duckiebot")
                    self.deleteMovingBot(botID)

                else : 
                    rospy.loginfo("["+self.node_name+"] Something unexpected has heppend in updateLastNeighbor")




    def releaseChargerSpot(self,tagID):
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
            if tagID not in list(movingAprilTags.keys()) :
                self.movingAprilTags[tagID]['first_neighbor'] = self.NearestNeighbor(tagPose)


            self.movingAprilTags[tagID]['position'] = tagPose
            self.movingAprilTags[tagID]['timestamp'] = current_time
            self.movingAprilTags[tagID]['last_neighbor'] = self.NearestNeighbor(tagPose)

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

#!/usr/bin/env python3
# Copyright <2022> <Tjark Schuette (tschuette@atb-potsdam.de)>

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

#     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#     Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from sensor_msgs.msg import RelativeHumidity, NavSatFix


class RfidReader():
    
    def __init__(self):        

        # initialise storage space for 
        self.current_pos = NavSatFix()

        # hook the first subscriber to our rfid-callback
        rospy.Subscriber("/rfid_detections", RelativeHumidity, self.rfid_callback, queue_size=1)
        
        # hook the first subscriber to the fix-callback
        rospy.Subscriber("/uav1/fix", NavSatFix, self.gps_callback)
        
        # bool to avoid old latched message
        self.init = True
   
    # RFID detection callback 
    def rfid_callback(self, message : RelativeHumidity):
        # skip first message (old latched)
        if self.init:
            return
        # print out the frame_id (in this case sensor_id) and the humidity value
        """
        YOUR CODE GOES BELOW THIS PART 
        
        add the latest GPS position to the data printout
        
        since variables are only valid within function scope (inside a function) we will use an object attribute that is set in the gps-callback
        
        object attributes can be accessed through "self.attribute" (e.g. print(self.init) )
        
        HINT: take a look at the printout in the gps-callback to get the syntax 
        
        """
        # print RFID-sensor info to the screen
        rospy.loginfo("\n\n Read RFID-Sensor! Sensor: %s Humidity: %f \n", message.header.frame_id, message.relative_humidity)
    
    # GPS-position (fix) message callback 
    def gps_callback(self, message : NavSatFix):
        
        # let other callbacks know that gps is available
        if self.init:
            self.init = False
        
        # print the current position every two seconds (not for every message)
        rospy.loginfo_throttle(2.0, "Read GPS Position. Lat: %f Long: %f \n", message.latitude, message.longitude)
        
        # store the position in a object attribute
        self.current_pos = message

    def run(self):

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    RFID_reader = RfidReader()
    RFID_reader.run()
    
    
    
    
    
    
    
    
    
        
    
    
    
    
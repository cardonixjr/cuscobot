import rospy, time
from std_msgs.msg import Int32, Int16, String, Empty
#import plotly.graph_objects as go

# MG49 commands
SET_SPEED_RIGHT = chr(100)
SET_SPEED_LEFT = chr(101)
SET_SPEED = chr(102)
GET_LEFT_ENCODER = chr(105)
GET_RIGHT_ENCODER = chr(106)
RESET_ENCODER = chr(114)


class DeadReckoningOdom():
    def __init__(self):
        # Node name to this class
        self.nodeName = "DeadReckoningOdom"


        ########## ROS DEFINITION ##########
        # ROS node
        rospy.init_node(self.nodeName, anonymous = True)
        self.nodeName = rospy.get_name()
        rospy.loginfo(f"The node - {self.nodeName} has started")

        # Subscribers for receiving encoder readings
        rospy.Subscriber("left_encoder_pulses", Int32, self.callBackFunctionLeftEncoder)
        rospy.Subscriber("right_encoder_pulses", Int32, self.callBackFunctionLeftEncoder)
        
        
        self.leftPublisher = rospy.Publisher("left_wheel_pwm", Int32, queue_size=5)
        self.rightPublisher = rospy.Publisher("right_wheel_pwm", Int32, queue_size=5)

    def callBackFunctionLeftEncoder(self, message1):
        print(message1)


    def mainLoop(self):
        num = 0
        while not rospy.is_shutdown():
            
            # self.calculateUpdate()
            self.leftPublisher.publish(128+num)
            self.rightPublisher.publish(128-num)
            #num+=1
            time.sleep(0.5)
   

if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()

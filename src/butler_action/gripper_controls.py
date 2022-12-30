import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput, Robotiq3FGripperRobotInput


class GripperControls(object):
    def __init__(self):
        # nh_ = rospy.init_node('gripper_controls')
        self.gripper_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1)
        self.command = Robotiq3FGripperRobotOutput();
        # self.command = genCommand('a', self.command)
        # self.gripper_pub.publish(self.command)

    def gripper_command(self, char):
        self.command = genCommand(char, self.command)
        self.gripper_pub.publish(self.command)
        rospy.sleep(0.1)
        
    def activate_gripper(self):
        """Activate the gripper."""
        self.gripper_command('r')
        self.gripper_command('a')
        while not rospy.is_shutdown():
            status = rospy.wait_for_message('Robotiq3FGripperRobotInput', Robotiq3FGripperRobotInput)
            if status.gACT != 1: #Means gripper not activated yet or is in reset mode
                continue
            if status.gGTO != 1: #Means gripper is moving or still performing activation
                continue
            if status.gIMC != 3: #Means gripper is still performing activation
                continue
            if status.gSTA != 3: #Means gripper is still performing activation
                continue
            if status.gDTA != 3: #Means Finger A is still moving or is not at requested position
                continue
            if status.gDTB != 3: #Means Finger B is still moving or is not at requested position
                continue
            if status.gDTC != 3: #Means Finger C is still moving or is not at requested position
                continue
            if status.gFLT != 0: #Means gripper is in fault state
                continue
            break
        print("Done Activation")
        return True
    
    def pinch_mode(self):
        """Activate the gripper Pinch Mode."""
        while not rospy.is_shutdown():
            self.gripper_command('p')
            print("Trying to Pinch")
            status = rospy.wait_for_message('Robotiq3FGripperRobotInput', Robotiq3FGripperRobotInput)
            if status.gMOD != 1:
                print("Gripper not in Pinch Mode")
                continue
            if status.gPOS < 219:
                print("Gripper scissor not in position")
                continue
            if status.gFLT != 0: #Means gripper is in fault state
                print("Gripper in Fault")
                continue
            break
        print("Done pinch_mode")
        return True
    
    def move_gripper(self, pos):
        while not rospy.is_shutdown():
            self.gripper_command(pos)
            print("Trying to move gripper to pos == {}".format(pos))
            status = rospy.wait_for_message('Robotiq3FGripperRobotInput', Robotiq3FGripperRobotInput)
            if status.gACT != 1: #Means gripper not activated yet or is in reset mode
                while not self.activate_gripper():
                    print("Trying to activate first before moving gripper")
                    continue
            if status.gGTO != 1: #Means gripper is moving or still performing activation
                print("Gripper Moving")
                continue
            if status.gDTA != 3: #Means Finger A is still moving or is not at requested position
                print("Finger A is Moving")
                continue
            if status.gDTB != 3: #Means Finger B is still moving or is not at requested position
                print("Finger B is Moving")
                continue
            if status.gDTC != 3: #Means Finger C is still moving or is not at requested position
                print("Finger C is Moving")
                continue
            if status.gFLT != 0: #Means gripper is in fault state
                print("Gripper is at Fault")
                continue
            break
        print("Done Moving Gripper")
        return True

def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if char == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0

    if char == 'p':
        command.rMOD = 1

    if char == 'w':
        command.rMOD = 2

    if char == 's':
        command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

import smach
import smach_ros

from math import pi, copysign, atan2, sin, cos

from pid import PID

class FollowLine(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['found_basket', 'lost_line', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_heave(0)
        while not rospy.is_shutdown():
            if self.cntr.object_found('BASKET'):
                return 'found_basket'
            elif not self.cntr.object_found('LINE'):
                if (self.cntr.last_msgs['LINE'].twist.linear.y < 0.25 and
                    self.cntr.last_msgs['LINE'].twist.angular.z < 30 * pi / 180):
                    return 'aborted'
                else:
                    return 'lost_line'
            else:
                self.cntr.follow_line()
                self.cntr.rate.sleep()
        return 'aborted'

class ReturnOnLine(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['found_line', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.cntr.object_found('LINE'):
                return 'found_line'
            elif (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(5):
                return 'aborted'
            else:
                self.cntr.search_for_line()
                self.cntr.rate.sleep()
        return 'aborted'

class MoveStraight(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['found_line', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_yaw_sp(self.cntr.last_msgs['ROBOT'].twist.angular.z)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(20):
                return 'aborted'
            elif self.cntr.object_found('LINE'):
                return 'found_line'
            else:
                self.cntr.move_straight()
                self.cntr.rate.sleep()
        return 'aborted'

class StayOnBasket(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_heave(0)
        self.cntr.set_yaw_sp(self.cntr.last_msgs['ROBOT'].twist.angular.z)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(3):
                return 'succeeded'
            elif not self.cntr.object_found('BASKET'):
                return 'aborted'
            else:
                self.cntr.follow_basket()
                self.cntr.rate.sleep()
        return 'aborted'

class Submerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(15):
                return 'succeeded'
            else:
                self.cntr.set_heave(-30)
                self.cntr.follow_basket()
                self.cntr.rate.sleep()
        return 'aborted'

class Emerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if ((rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(20) or
                (not self.cntr.object_found('BASKET')) and self.cntr.object_found('LINE')):
                return 'aborted'
            else:
                self.cntr.emerge()
                self.cntr.rate.sleep()
        return 'aborted'

class Stop(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['found_line', 'found_basket', 'aborted'])
        self.cntr = c
        
    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_heave(0)
        while not rospy.is_shutdown():
            if self.cntr.object_found('LINE'):
                return 'found_line'
            elif self.cntr.object_found('BASKET'):
                return 'found_basket'
            else:
                self.cntr.set_effort(0,0,0)
                self.cntr.rate.sleep()
        return 'aborted'

class Controller():
    def __init__(self):
        if not (rospy.has_param('~surge_pid') and rospy.has_param('~sway_pid') 
                and rospy.has_param('~yaw_pid')):
            rospy.logerr("Didn't find pid configuration")
            raise rospy.ROSException
        
        self._surge_pid = PID(rospy.get_param('~surge_pid'))
        self._sway_pid = PID(rospy.get_param('~sway_pid'))
        self._yaw_pid = PID(rospy.get_param('~yaw_pid'))

        self.rate = rospy.Rate(5)

        self.last_msgs = {'ROBOT': TwistStamped(), 
                          'LINE': TwistStamped(), 
                          'BASKET': TwistStamped()}

        rospy.Subscriber('robot_pose', TwistStamped, self.msg_callback, 'ROBOT')
        rospy.Subscriber('line_pose', TwistStamped, self.msg_callback, 'LINE')
        rospy.Subscriber('basket_pose', TwistStamped, self.msg_callback, 'BASKET')

        self._surge_publisher = rospy.Publisher('surge', Float64, queue_size=1)
        self._sway_publisher = rospy.Publisher('sway', Float64, queue_size=1)
        self._yaw_publisher = rospy.Publisher('yaw', Float64, queue_size=1)
        self._heave_publisher = rospy.Publisher('heave', Float64, queue_size=1)
        
        self.state_change_time = rospy.Time.now()
        
    def msg_callback(self, msg, sender):
        self.last_msgs[sender] = msg

    def set_effort(self, surge, sway, yaw):
        self._surge_publisher.publish(surge)
        self._sway_publisher.publish(sway)
        self._yaw_publisher.publish(yaw)

    def move_straight(self):
        yaw_error = self._yaw_sp - self.last_msgs['ROBOT'].twist.angular.z
        yaw_error = atan2(sin(yaw_error), cos(yaw_error))
        yaw_effort = self._yaw_pid.update(yaw_error)
        self.set_effort(30, 0, yaw_effort)
            
    def follow_line(self):
        surge_effort = self._surge_pid.update(pi/4 - abs(self.last_msgs['LINE'].twist.angular.z) / (pi/4))
        sway_effort = self._sway_pid.update(self.last_msgs['LINE'].twist.linear.y)
        yaw_error = atan2(sin(self.last_msgs['LINE'].twist.angular.z),  # error -> [-pi, pi]
                          cos(self.last_msgs['LINE'].twist.angular.z))  
        yaw_effort = self._yaw_pid.update(yaw_error)
        self.set_effort(surge_effort, sway_effort, yaw_effort)
    
    def follow_basket(self):
        surge_effort = -self._surge_pid.update(self.last_msgs['BASKET'].twist.linear.x)
        sway_effort = self._sway_pid.update(self.last_msgs['BASKET'].twist.linear.y)
        yaw_error = self._yaw_sp - self.last_msgs['ROBOT'].twist.angular.z
        yaw_error = atan2(sin(yaw_error), cos(yaw_error))
        yaw_effort = self._yaw_pid.update(yaw_error)
        self.set_effort(surge_effort, sway_effort, yaw_effort)

    def emerge(self):
        self.set_heave(20)
        sway_effort = copysign(30, self.last_msgs['LINE'].twist.linear.y)
        yaw_error = self._yaw_sp - self.last_msgs['ROBOT'].twist.angular.z
        yaw_error = atan2(sin(yaw_error), cos(yaw_error))
        yaw_effort = self._yaw_pid.update(yaw_error)
        self.set_effort(30, sway_effort, yaw_effort)

    def search_for_line(self):
        self.set_effort(0, 0.0, 0.0)
        # self.set_effort(-30, 0.0, 0.0)

    def set_heave(self, heave):
        self._heave_publisher.publish(heave)
        
    def set_yaw_sp(self, yaw_sp=0):
        self._yaw_sp = yaw_sp
        
    def object_found(self, obj):
        if (rospy.Time.now() - self.last_msgs[obj].header.stamp).to_sec() < 2:
            return True
        else:
            return False
        

if __name__ == '__main__':
    rospy.init_node('test')

    c = Controller()
    sm = smach.StateMachine(outcomes=['aborted'])
    with sm:
        smach.StateMachine.add('MOVE_STRAIGHT', MoveStraight(c),
                               transitions={'found_line': 'LINE_SM',
                                            'aborted': 'STOP'})
        smach.StateMachine.add('STOP', Stop(c),
                               transitions={'found_line': 'LINE_SM', 
                                            'found_basket': 'BASKET_SM',
                                            'aborted': 'aborted'})

        sm_line = smach.StateMachine(outcomes=['aborted', 'found_basket'])
        with sm_line:
            smach.StateMachine.add('FOLLOW_LINE', FollowLine(c), 
                                   transitions={'lost_line': 'RETURN_ON_LINE',
                                                'found_basket': 'found_basket',
                                                'aborted': 'aborted'}) 
            smach.StateMachine.add('RETURN_ON_LINE', ReturnOnLine(c), 
                                   transitions={'found_line': 'FOLLOW_LINE',
                                                'aborted': 'aborted'}) 

        sm_basket = smach.StateMachine(outcomes=['aborted'])
        with sm_basket:
            smach.StateMachine.add('STAY_ON_BASKET', StayOnBasket(c),
                                   transitions={'succeeded': 'SUBMERGE',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('SUBMERGE', Submerge(c),
                                   transitions={'succeeded': 'EMERGE',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('EMERGE', Emerge(c),
                                   transitions={'aborted': 'aborted'})

        smach.StateMachine.add('LINE_SM', sm_line, 
                               transitions={'aborted': 'STOP',
                                            'found_basket': 'BASKET_SM'})

        smach.StateMachine.add('BASKET_SM', sm_basket, 
                               transitions={'aborted': 'STOP'}) 
        
    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # sis.stop()

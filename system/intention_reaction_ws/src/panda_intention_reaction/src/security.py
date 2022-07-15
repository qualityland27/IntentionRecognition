from geometry_msgs.msg import Pose, PoseStamped
import rospy


class PandaMovementSecurity:
    def __init__(self):
        self.min_x = 0.2
        self.min_y = -0.3
        self.min_z = 0.1

        self.max_x = 0.7
        self.max_y = 0.3
        self.max_z = 0.9
        self.ws = [self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z]
    
    def pose_within_ws(self, pose):
        '''
        Check if the given pose is within the secure workspace.
        
        :param pose: The pose of the target position in world frame.
        :return: Bool if target position is within secure space.
        '''
        if self.min_x <= pose.position.x <= self.max_x:
            if self.min_y <= pose.position.y <= self.max_y:
                if self.min_z <= pose.position.z <= self.max_z:
                    return True
        
        return False

    def positions_are_the_same(self, pose1, pose2, tollerance=0.1):
        '''
        Comparison of two position objects.
        '''
        x_dif = abs(pose1.position.x - pose2.position.x)
        y_dif = abs(pose1.position.y - pose2.position.y)
        z_dif = abs(pose1.position.z - pose2.position.z)

        if x_dif <= tollerance and y_dif <= tollerance and z_dif <= tollerance:
            return True
        else:
            return False

    def add_collision_objects(self, scene, planning_frame, table=True, camera=True, desktops=True):
        '''
        Adding Collision Objects to MoveIt!
        '''
        def ensure_updates_are_made(box_name):
            start = rospy.get_time()
            seconds = rospy.get_time()
            timeout = 5.0

            while (seconds - start < timeout) and not rospy.is_shutdown():
                # Test if the box is in attached objects
                attached_objects = scene.get_attached_objects([box_name])
                is_attached = len(attached_objects.keys()) > 0

                # Test if the box is in the scene.
                # Note that attaching the box will remove it from known_objects
                is_known = box_name in scene.get_known_object_names()

                # Test if we are in the expected state
                if (True == is_attached) and (True == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

                # If we exited the while loop without returning then we timed out
                return False

        box0 = PoseStamped()
        box0.header.frame_id = planning_frame
        box0.pose.position.x = 0.
        box0.pose.position.y = 0.
        box0.pose.position.z = 0.
        box0_name = "test"
        scene.add_box(box0_name, box0, (0., 0., 0.))
        ensure_updates_are_made(box0_name)

        if table:
            box = PoseStamped()
            box.header.frame_id = planning_frame
            box.pose.position.x = 0.6
            box.pose.position.y = 0.
            box.pose.position.z = -0.1
            box_name = "table"
            scene.add_box(box_name, box, (0.8, 0.6, 0.18))
            ensure_updates_are_made(box_name)
            print("table")
        
        if camera:
            box2 = PoseStamped()
            box2.header.frame_id = planning_frame
            box2.pose.position.x = -0.3
            box2.pose.position.y = 0.
            box2.pose.position.z = 1.6
            box2_name = "camera"
            scene.add_box(box2_name, box2, (0.1, 0.2, 1.0))
            ensure_updates_are_made(box2_name)
        
        if desktops:
            box3 = PoseStamped()
            box3.header.frame_id = planning_frame
            box3.pose.position.x = 0.
            box3.pose.position.y = 0.6
            box3.pose.position.z = .25
            box3_name = "desktops"
            scene.add_box(box3_name, box3, (2.0, 0.2, 0.5))
            ensure_updates_are_made(box3_name)
        
        return scene, planning_frame
        
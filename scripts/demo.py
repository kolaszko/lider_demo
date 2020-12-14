from arm_planner.arm_planner import ArmPlanner, Point, Quaternion
import rospy
import pickle


def move_to_camera(robot):
    p = Point(-0.254879010125, -0.284953690094, 0.617899606263)
    q = Quaternion(0.707140566901, 0.707011938205, -0.00654724248163, 0.00659329167968)

    robot.move_pose(p, q)

if __name__ == '__main__':

    rospy.init_node('demo')
    rospy.sleep(3.)
    right_arm = ArmPlanner('right_arm', 'right_arm')
    right_arm.group.set_max_acceleration_scaling_factor(.2)
    right_arm.group.set_max_velocity_scaling_factor(.2)

    # move_to_camera(right_arm)
    rospy.sleep(2.)
    path = None
    with open('curve_2_full_run.pickle', 'rb') as f:
        path = pickle.load(f)

    counter = 0
    for i in path:
        p = Point(i.pose.position.x, i.pose.position.y, i.pose.position.z)
        q = Quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)
        right_arm.move_pose(p, q)
        if counter == 0:
            print('Sleeeping...')
            rospy.sleep(10.0)
        else:
            rospy.sleep(1.0)

        counter += 1
        # if counter == 5:
        #     break
        print(p)
        print(q)



import rospy
import pickle

from arm_planner.arm_planner import ArmPlanner


if __name__ == '__main__':
    rospy.init_node('data_collector')
    rospy.sleep(2.)
    right_arm = ArmPlanner('right_arm', 'right_arm')

    ds = []

    def save_ds():
        with open('curve_2_full.pickle', 'wb') as f:
            print('Saving...')
            pickle.dump(ds, f)


    rospy.on_shutdown(save_ds)

    while not rospy.is_shutdown():
        p = right_arm.group.get_current_pose()
        ds.append(p)
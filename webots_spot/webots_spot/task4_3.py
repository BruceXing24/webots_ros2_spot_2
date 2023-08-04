import rclpy 
from  .gripper_client import GripperClient as Gripper
from .pickandplace.moveit_ik import MoveitIKClientAsync as IK
from .pickandplace.moveit_fk_action import MoveGroupActionClient as Moveit
from .pickandplace.nav2_action import NavigateToPoseActionClient as Nav2GoTo
import time


def main():
    rclpy.init()
    gripper = Gripper('gripper')
    nav2 = Nav2GoTo('nav2')
    moveit = Moveit()
    # 0.38 3.29 0.008
    target_p = [0.32,3.45,0.008]
    # print(target_p)
    target_o = [0.,0.,0.707,0.707]
    nav2.send_goal(target_p,target_o)
    print('go to pick point')
    while not nav2.get_nav2_status():
        rclpy.spin_once(nav2)

    print('start to move arm')
    ik_solver = IK()
    target_angles = ik_solver.send_request('P')
    ik_solver.destroy_node()

    moveit.send_goal(target_angles)
    while not moveit.get_moveit_status():
        rclpy.spin_once(moveit)

    time.sleep(2.0)    
    print('start to grap')
    gripper.close_request()
    time.sleep(2.0)
    print('gripper closed')


    print('start to move arm to PlaceBox')
    ik_solver = IK()
    target_angles = ik_solver.send_request('PlaceBox')
    ik_solver.destroy_node()
    moveit.send_goal(target_angles)
    while not moveit.get_moveit_status():
        rclpy.spin_once(moveit)
    print('start to open gripper')
    gripper.open_request()
    time.sleep(2.0)


    gripper.destroy_node()
    nav2.destroy_node()
    moveit.destroy_node()

    rclpy.shutdown()





if __name__ == '__main__':
    main()
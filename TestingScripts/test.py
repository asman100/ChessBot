#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String


def gantry_state_callback(msg):
    # Callback to receive gantry state updates
    rospy.loginfo(f"Gantry status: {msg.data}")
    if msg.data == "Homing Done":
        rospy.loginfo("Homing operation is complete.")
    elif msg.data == "Movement Done":
        rospy.loginfo("Movement operation is complete.")


def move_gantry(x, y):
    # Function to publish the gantry position
    position_command = f"{x},{y}"
    rospy.loginfo(f"Moving gantry to position: {position_command}")
    position_pub.publish(position_command)


def control_magnet(state):
    # Function to publish the magnet control command
    magnet_state = Bool()
    magnet_state.data = state
    rospy.loginfo(f"Setting magnet to: {'ON' if state else 'OFF'}")
    magnet_pub.publish(magnet_state)


if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node("gantry_controller", anonymous=True)

        # Publishers
        magnet_pub = rospy.Publisher("/magnet", Bool, queue_size=10)
        position_pub = rospy.Publisher("/position", String, queue_size=10)

        # Subscriber to gantry state
        rospy.Subscriber("/gantrystate", String, gantry_state_callback)

        # Allow some time for connections to be established
        rospy.sleep(2)

        # Example usage: Control gantry and magnet
        while not rospy.is_shutdown():
            # Move the gantry to position (X, Y)
            x_position = input("Enter X position (mm): ")
            y_position = input("Enter Y position (mm): ")
            move_gantry(x_position, y_position)

            # Wait for gantry to reach its position
            rospy.sleep(5)  # Adjust sleep time based on gantry speed

            # Example: Turn on the magnet
            control_magnet(True)
            rospy.sleep(2)

            # Example: Turn off the magnet
            control_magnet(False)
            rospy.sleep(2)

            rospy.loginfo("Finished current cycle. Ready for new command.")

    except rospy.ROSInterruptException:
        pass

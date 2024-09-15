#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

magnet_pub = rospy.Publisher("/magnet", Bool, queue_size=10)
position_pub = rospy.Publisher("/position", String, queue_size=10)
bot_status_pub = rospy.Publisher("/botmovestatus", String, queue_size=10)
botstate = "Idle"
capture = False


def gantry_state_callback(msg):
    global botstate
    rospy.loginfo(f"Gantry status: {msg.data}")
    if msg.data == "Homing Done":
        botstate = "Idle"
        rospy.loginfo("Homing operation is complete.")
    elif msg.data == "Done":
        botstate = "Idle"
        rospy.loginfo("Movement operation is complete.")


def move_gantry(goal):
    # Function to publish the gantry position
    position_command = goal
    rospy.loginfo(f"Moving gantry to position: {position_command}")
    position_pub.publish(position_command)


def control_magnet(state):
    # Function to publish the magnet control command
    magnet_state = Bool()
    magnet_state.data = state
    rospy.loginfo(f"Setting magnet to: {'ON' if state else 'OFF'}")
    magnet_pub.publish(magnet_state)


def botmove_callback(msg):
    # Callback to receive bot move commands
    rospy.loginfo(f"Received bot move: {msg.data}")
    botmove = msg.data
    moveplanner(botmove)


def moveplanner(botmove):
    global botstate, capture
    pos = posextractor(botmove)
    if botstate != "Idle":
        rospy.logwarn("Gantry is busy. Ignoring bot move command.")
        return

    botstate = "Moving"
    goal = pos[1]
    start = pos[0]
    if capture:

        goal_string = f"{goal[0]},{goal[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        control_magnet(True)
        rospy.sleep(1)
        goal_string = f"{goal[0]},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        goal_string = f"{0},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        control_magnet(False)

    goal_string = f"{start[0]},{start[1]}"
    move_gantry(goal_string)
    while botstate == "Moving":
        pass
    control_magnet(True)
    goal_string = f"{start[0]+25},{start[1]}"
    move_gantry(goal_string)
    while botstate == "Moving":
        pass
    goal_string = f"{start[0]+25},{goal[1]+25}"
    move_gantry(goal_string)
    while botstate == "Moving":
        pass
    goal_string = f"{goal[0]},{goal[1]+25}"
    move_gantry(goal_string)
    while botstate == "Moving":
        pass
    goal_string = f"{goal[0]},{goal[1]}"
    move_gantry(goal_string)
    while botstate == "Moving":
        pass
    rospy.sleep(10)
    control_magnet(False)
    botstate = "Idle"
    bot_status_pub.publish("confirmed")


def posextractor(move):
    global capture
    # Split the move string into UCI part and capture part
    move_parts = move.split()  # This splits the string into [uci_move, capture_info]

    uci_move = move_parts[0]  # Extract the UCI move (e.g., e2e4)
    capture = move_parts[1] == "True"  # Extract the capture info as a boolean

    # Extract start and end squares from the UCI move
    start_square = uci_move[:2]  # First two characters are the start square (e.g., e2)
    end_square = uci_move[-2:]  # Last two characters are the end square (e.g., e4)

    if capture == True:
        # If a capture is happening, extract the capture square
        capture_square = end_square

        rospy.loginfo(f"Capture square: {capture_square}")
    startpos = chess_position_to_coordinates(start_square)
    endpos = chess_position_to_coordinates(end_square)
    movearray = [startpos, endpos]
    return movearray


def chess_position_to_coordinates(pos):
    J = 50
    Z = 80
    # Map the file (column) letters to x-axis positions (starting from 'h' as 0)
    file_map = {"a": 7, "b": 6, "c": 5, "d": 4, "e": 3, "f": 2, "g": 1, "h": 0}

    # Extract the file (letter) and rank (number) from the position
    file = pos[0]  # The letter part, e.g., 'e'
    rank = int(pos[1])  # The number part, e.g., '2'

    # Calculate x and y based on the file and rank
    x = file_map[file] * 50  # Each square is 50mm wide
    y = (rank - 1) * 50  # Rank starts at 1, but we need to map it to 0-based index

    # Add the constant offset J to both x and y
    x += J
    y += Z

    return [x, y]


def main():

    # Initialize the ROS node
    rospy.init_node("gantry_controller", anonymous=True)

    # Subscriber to gantry state
    rospy.Subscriber("/gantrystate", String, gantry_state_callback)
    rospy.Subscriber("/botmoves", String, botmove_callback)

    # Allow some time for connections to be established
    rospy.sleep(2)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

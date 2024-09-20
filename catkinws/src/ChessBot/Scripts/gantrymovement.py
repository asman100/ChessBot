#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String, Int32MultiArray
import threading

magnet_pub = rospy.Publisher("/magnet", Bool, queue_size=10)
position_pub = rospy.Publisher("/position", String, queue_size=10)
bot_status_pub = rospy.Publisher("/botmovestatus", String, queue_size=10)
botstate = "Idle"
capture = False
castling = False
Home = f"{10},{200}"

# Global variables for sensor readings
sensor_lock = threading.Lock()
sensor_readings = []  # Flat list of 64 elements


data_updated = threading.Event()


def sensor_array_callback(msg):
    global sensor_readings
    with sensor_lock:
        sensor_readings = list(msg.data)
        data_updated.set()  # Signal that new data is available


def check_piece_placement(square):
    # Wait for new data if necessary
    if not data_updated.is_set():
        data_updated.wait(timeout=1)  # Wait up to 1 second
    with sensor_lock:
        index = chess_square_to_index(square)
        sensor_value = sensor_readings[index]
    data_updated.clear()  # Reset the event for the next update
    return sensor_value == 1


def chess_square_to_index(square):
    # Method 3: index = (7 - rank_index) * 8 + file_index
    file_map = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "f": 5, "g": 6, "h": 7}
    file = square[0]
    rank = int(square[1])  # Ranks from 1 to 8

    file_index = file_map[file]
    rank_index = rank - 1  # Adjusted mapping

    index = (7 - rank_index) * 8 + file_index
    return index


def gantry_state_callback(msg):
    global botstate
    rospy.loginfo(f"Gantry status: {msg.data}")
    if msg.data == "Homing Done":
        botstate = "Idle"
        rospy.loginfo("Homing operation is complete.")
    elif msg.data == "Done":
        botstate = "Idle"
        rospy.loginfo("Movement operation is complete.")
    elif msg.data == "Moving":
        botstate = "Moving"


def move_gantry(goal):
    position_command = goal
    rospy.loginfo(f"Moving gantry to position: {position_command}")
    position_pub.publish(position_command)


def control_magnet(state):
    magnet_state = Bool()
    magnet_state.data = state
    rospy.loginfo(f"Setting magnet to: {'ON' if state else 'OFF'}")
    magnet_pub.publish(magnet_state)


def botmove_callback(msg):
    rospy.loginfo(f"Received bot move: {msg.data}")
    botmove = msg.data
    moveplanner(botmove)


def attempt_piece_placement(goal_pos, end_square):
    max_attempts = 5
    step_size = 10  # Adjust step size as needed
    x0, y0 = goal_pos[0], goal_pos[1]
    offsets = [
        (0, 0),
        (step_size, 0),
        (-step_size, 0),
        (0, step_size),
        (0, -step_size),
        (step_size, step_size),
        (-step_size, -step_size),
        (step_size, -step_size),
        (-step_size, step_size),
    ]
    for attempt in range(max_attempts):
        dx, dy = offsets[attempt % len(offsets)]
        new_x = x0 + dx
        new_y = y0 + dy
        goal_string = f"{new_x},{new_y}"
        rospy.loginfo(f"Attempt {attempt+1}: Moving to adjusted position {goal_string}")
        move_gantry(goal_string)
        # Wait for gantry to finish moving
        while botstate == "Moving":
            rospy.sleep(0.1)  # Avoid busy waiting
        rospy.sleep(1)
        control_magnet(False)
        rospy.sleep(2)
        # Wait for new sensor data
        data_updated.wait(timeout=1)
        if check_piece_placement(end_square):
            rospy.loginfo("Piece correctly placed.")
            return True
        else:
            rospy.logwarn("Piece not detected at adjusted position.")
            control_magnet(True)
            rospy.sleep(2)
    rospy.logwarn("Failed to place piece correctly after adjustments.")
    return False


def moveplanner(botmove):
    global botstate, capture, castling
    pos, start_square, end_square = posextractor(botmove)
    if botstate != "Idle":
        rospy.logwarn("Gantry is busy. Ignoring bot move command.")
        return
    control_magnet(False)
    rospy.sleep(2)
    botstate = "Moving"
    goal = pos[1]
    start = pos[0]
    if castling:
        # Determine whether it's kingside or queenside castling
        if start_square[0] == "e" and end_square[0] == "g":
            # Kingside castling
            king_start = pos[0]  # King's current position (e1 or e8)
            king_end = chess_position_to_coordinates(
                "g" + start_square[1]
            )  # King's new position (g1 or g8)
            rook_start = chess_position_to_coordinates(
                "h" + start_square[1]
            )  # Rook's current position (h1 or h8)
            rookendsquare = "f" + start_square[1]
            rook_end = chess_position_to_coordinates(
                "f" + start_square[1]
            )  # Rook's new position (f1 or f8)
        elif start_square[0] == "e" and end_square[0] == "c":
            # Queenside castling
            king_start = pos[0]  # King's current position (e1 or e8)
            king_end = chess_position_to_coordinates(
                "c" + start_square[1]
            )  # King's new position (c1 or c8)
            rook_start = chess_position_to_coordinates(
                "a" + start_square[1]
            )  # Rook's current position (a1 or a8)
            rookendsquare = "d" + start_square[1]
            rook_end = chess_position_to_coordinates(
                "d" + start_square[1]
            )  # Rook's new position (d1 or d8)

        # Move the king
        goal_string = f"{10},{king_start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{king_start[0]},{king_start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        control_magnet(True)
        rospy.sleep(2)
        goal_string = f"{king_start[0]},{king_start[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{king_end[0]},{king_end[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{king_end[0]},{king_end[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(False)
        success = attempt_piece_placement(king_end, end_square)
        if not success:
            rospy.logwarn("Failed to place piece correctly.")
        # Move the rook
        goal_string = f"{10},{rook_start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{rook_start[0]},{rook_start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        control_magnet(True)
        rospy.sleep(2)
        goal_string = f"{rook_start[0]},{rook_start[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{rook_end[0]},{rook_end[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{rook_end[0]},{rook_end[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        success = attempt_piece_placement(rook_end, rookendsquare)
        if not success:
            rospy.logwarn("Failed to place piece correctly.")
        control_magnet(False)

        rospy.sleep(2)
        move_gantry(Home)
        botstate = "Idle"
        bot_status_pub.publish("confirmed")
        return
    elif capture:
        # Capture sequence...
        goal_string = f"{10},{goal[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{goal[0]},{goal[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(True)
        rospy.sleep(1)
        goal_string = f"{goal[0]},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{0},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(False)
        rospy.sleep(2)
        goal_string = f"{start[0]},{start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(True)
        rospy.sleep(2)
        goal_string = f"{start[0]+25},{start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{start[0]+25},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{goal[0]},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{goal[0]},{goal[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(False)
        rospy.sleep(2)
        # Attempt to place piece correctly
        success = attempt_piece_placement(goal, end_square)
        if not success:
            rospy.logwarn("Failed to place piece correctly.")
        move_gantry(Home)
        botstate = "Idle"
        bot_status_pub.publish("confirmed")
    else:
        # Normal move sequence
        goal_string = f"{10},{start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{start[0]},{start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        control_magnet(True)
        rospy.sleep(2)
        goal_string = f"{start[0]+25},{start[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{start[0]+25},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{goal[0]},{goal[1]+25}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        goal_string = f"{goal[0]},{goal[1]}"
        move_gantry(goal_string)
        while botstate == "Moving":
            pass
        rospy.sleep(2)
        control_magnet(False)
        rospy.sleep(2)
        # attempt to place the piece
        success = attempt_piece_placement(goal, end_square)
        if not success:
            rospy.logwarn("Failed to place piece correctly.")
        move_gantry(Home)
        botstate = "Idle"
        bot_status_pub.publish("confirmed")


def posextractor(move):
    global capture, castling
    move_parts = move.split()
    uci_move = move_parts[0]
    capture = move_parts[1] == "True"
    castling = move_parts[2] == "True"
    start_square = uci_move[:2]
    end_square = uci_move[-2:]
    startpos = chess_position_to_coordinates(start_square)
    endpos = chess_position_to_coordinates(end_square)
    movearray = [startpos, endpos]
    return movearray, start_square, end_square


def chess_position_to_coordinates(pos):
    J = 54
    Z = 80
    file_map = {"a": 7, "b": 6, "c": 5, "d": 4, "e": 3, "f": 2, "g": 1, "h": 0}
    file = pos[0]
    rank = int(pos[1])
    x = file_map[file] * 50 + J
    y = (rank - 1) * 50 + Z
    return [x, y]


def main():
    rospy.init_node("gantry_controller", anonymous=True)
    rospy.Subscriber("/gantrystate", String, gantry_state_callback)
    rospy.Subscriber("/botmoves", String, botmove_callback)
    rospy.Subscriber("/chessboard", Int32MultiArray, sensor_array_callback)
    rospy.sleep(2)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int32MultiArray
import time

# Simulated chessboard states for en passant
# 0: Empty square
# 1: A piece is present

# Initial board state: standard opening
initial_board = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 1: White pawn moves from e2 -> e4 (removal)
move_1_remove_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White pawn removed from e2
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 1: White pawn moves from e2 -> e4 (placement)
move_1_place_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White pawn placed on e4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (e2 pawn moved to e4)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 2: Black pawn moves from d7 -> d5 (removal)
move_2_remove_black_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black pawn removed from d7
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White pawn on e4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 2: Black pawn moves from d7 -> d5 (placement)
move_2_place_black_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black pawn placed on d5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White pawn still on e4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 3: White pawn captures d5 via en passant (removal of white pawn from e4)
move_3_remove_white_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black pawn on d5, White pawn removed from e4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]

# Move 3: White pawn captures d5 via en passant (removal of black pawn)
move_3_remove_black_pawn = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 1, 0, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]
move_4 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 0, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 1, 0, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]
move_4cont = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 0, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 1, 1, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]
move_5 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 0, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 0, 1, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]
move_51 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 0, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]
move_52 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 0, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 6
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black pawn removed from d5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White pawn now on d5 (en passant capture)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces (row 1)
]


# Function to simulate publishing board states and end turn events
def simulate_moves():
    rospy.init_node("chessboard_test_simulator", anonymous=True)
    board_pub = rospy.Publisher("chessboard", Int32MultiArray, queue_size=10)
    end_turn_pub = rospy.Publisher("end_turn", Bool, queue_size=10)

    # Wait for everything to initialize
    rospy.sleep(1)

    # Initial board state
    publish_board(board_pub, initial_board)
    rospy.sleep(2)

    # Move 1: White pawn e2 -> e4 (removal)
    rospy.loginfo("White pawn removed from e2")
    publish_board(board_pub, move_1_remove_pawn)
    rospy.sleep(2)

    # Move 1: White pawn e2 -> e4 (placement)
    rospy.loginfo("White pawn placed on e4")
    publish_board(board_pub, move_1_place_pawn)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)

    # Move 2: Black pawn d7 -> d5 (removal)
    rospy.loginfo("Black pawn removed from d7")
    publish_board(board_pub, move_2_remove_black_pawn)
    rospy.sleep(2)

    # Move 2: Black pawn d7 -> d5 (placement)
    rospy.loginfo("Black pawn placed on d5")
    publish_board(board_pub, move_2_place_black_pawn)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)

    # Move 3: White pawn captures via en passant (removal of white pawn from e4)
    rospy.loginfo("White pawn removed from e4")
    publish_board(board_pub, move_3_remove_white_pawn)
    rospy.sleep(2)

    # Move 3: White pawn captures via en passant (removal of black pawn from d5)
    rospy.loginfo("Black pawn removed from d5")
    publish_board(board_pub, move_3_remove_black_pawn)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.loginfo("Black pawn removed from d5")
    publish_board(board_pub, move_4)
    rospy.sleep(2)

    publish_board(board_pub, move_4cont)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)

    publish_board(board_pub, move_5)
    rospy.sleep(2)
    publish_board(board_pub, move_51)
    rospy.sleep(2)
    publish_board(board_pub, move_52)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)


def publish_board(pub, board_state):
    # Flatten the 8x8 matrix into a list
    flattened_board = [square for row in board_state for square in row]

    # Create a ROS message and publish the board state
    board_msg = Int32MultiArray(data=flattened_board)
    pub.publish(board_msg)


def publish_end_turn(pub):
    # Publish the end turn signal
    end_turn_msg = Bool(data=True)
    pub.publish(end_turn_msg)


if __name__ == "__main__":
    try:
        simulate_moves()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int32MultiArray, String
import time

# Simulated chessboard states
# 0: Empty square
# 1: A piece is present (starting configuration or after a move)
initialboard = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pieces, knight removed from g1
]
# Move 1: Remove White knight from g1
move_1_remove_knight = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces, knight removed from g1
]

# Move 2: Place White knight back on g1
move_2_place_knight = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pawns (row 7)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces, knight placed back on g1
]

# Move 3: Black makes a move (e.g., Black pawn d7 -> d6)
move_3_black_move0 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (d7 pawn moves to d6)
    [0, 0, 0, 0, 0, 0, 0, 0],  # Black's pawn now on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
move_3_black_move = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (d7 pawn moves to d6)
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black's pawn now on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 1, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
move_4_white_pawn_move0 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn now on e4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (e2 pawn moved to e4)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
# Move 4: White pawn moves (e.g., e2 -> e4)
move_4_white_pawn_move = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 1, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White's pawn now on e4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (e2 pawn moved to e4)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
move_black = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 0, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White's pawn now on e4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (e2 pawn moved to e4)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
move_black2 = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # Empty row 5
    [0, 0, 0, 0, 1, 0, 0, 0],  # White's pawn now on e4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (e2 pawn moved to e4)
    [1, 1, 1, 1, 1, 1, 0, 1],  # White's pieces (row 1)
]
# Move 5: Remove White bishop from f1
move_5_remove_bishop = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 0, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 0, 0, 1],  # White's bishop removed from f1 (row 1)
]

# Move 6: Place White bishop back on f1
move_6_place_bishop = [
    [1, 1, 1, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's pawn still on d6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 0, 0, 1],  # White's bishop placed back on f1 (row 1)
]

# Move 7: Black makes another move (e.g., Black knight b8 -> c6)
move_7_black_move0 = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8, knight removed from b8)
    [1, 1, 0, 0, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight moves to c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 0, 0, 1],  # White's pieces (row 1)
]

move_7_black_move = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8, knight removed from b8)
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight moves to c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 1, 0, 0, 1],  # White's pieces (row 1)
]

# Move 8: White kingside castling (King e1 -> g1, Rook h1 -> f1)
move_8_castling = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight on c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 0, 0, 0, 1],  # White's king on g1, rook on f1 (after castling)
]
move_8_castling1 = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight on c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 0, 0, 1, 1],  # White's king on g1, rook on f1 (after castling)
]
move_8_castling2 = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight on c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 0, 0, 1, 0],  # White's king on g1, rook on f1 (after castling)
]
move_8_castling3 = [
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pieces (row 8)
    [1, 1, 0, 1, 1, 1, 1, 1],  # Black's pawns (d6 still in place)
    [0, 0, 1, 1, 0, 0, 0, 0],  # Black's knight on c6
    [0, 0, 0, 0, 0, 0, 0, 0],  # White's pawn still on e4
    [0, 0, 0, 0, 1, 0, 0, 0],  # Empty row 4
    [0, 0, 0, 1, 0, 1, 0, 0],  # Empty row 3
    [1, 1, 1, 1, 0, 1, 1, 1],  # White's pawns (row 2)
    [1, 1, 1, 1, 0, 1, 1, 0],  # White's king on g1, rook on f1 (after castling)
]


# Function to simulate publishing board states and end turn events
def simulate_moves():
    rospy.init_node("chessboard_test_simulator", anonymous=True)
    board_pub = rospy.Publisher("chessboard", Int32MultiArray, queue_size=10)
    end_turn_pub = rospy.Publisher("end_turn", String, queue_size=10)

    # Wait for everything to initialize
    rospy.sleep(1)
    publish_board(board_pub, initialboard)
    rospy.sleep(2)
    """
    # Move 1: Remove White knight from g1
    rospy.loginfo("Remove White knight from g1")
    publish_board(board_pub, move_1_remove_knight)
    rospy.sleep(2)

    # Move 2: Place White knight back on g1
    rospy.loginfo("Place White knight back on g1")
    publish_board(board_pub, move_2_place_knight)
    rospy.sleep(2)

    publish_end_turn(end_turn_pub)
    rospy.sleep(2)
    # Move 3: Black plays a move (d7 -> d6)
    rospy.loginfo("Black plays a move (d7 -> d6)")
    publish_board(board_pub, move_3_black_move0)
    rospy.sleep(2)
    rospy.loginfo("Black plays a move (d7 -> d6)")
    publish_board(board_pub, move_3_black_move)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)

    # Move 4: White plays a pawn move (e2 -> e4)
    publish_board(board_pub, move_4_white_pawn_move0)
    rospy.sleep(2)
    rospy.loginfo("White plays a pawn move (e2 -> e4)")
    publish_board(board_pub, move_4_white_pawn_move)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)
    publish_board(board_pub, move_black)
    rospy.sleep(2)
    publish_board(board_pub, move_black2)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)
    # Move 5: Remove White bishop from f1
    rospy.loginfo("Remove White bishop from f1")
    publish_board(board_pub, move_5_remove_bishop)

    rospy.sleep(2)

    # Move 6: Place White bishop back on f1
    rospy.loginfo("Place White bishop back on f1")
    publish_board(board_pub, move_6_place_bishop)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)

    # Move 7: Black knight moves (b8 -> c6)
    rospy.loginfo("Black knight moves (b8 -> c6)")
    publish_board(board_pub, move_7_black_move)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)

    # Move 8: White performs kingside castling
    rospy.loginfo("White performs kingside castling")
    publish_board(board_pub, move_8_castling)
    rospy.sleep(2)
    publish_board(board_pub, move_8_castling1)
    rospy.sleep(2)
    publish_board(board_pub, move_8_castling2)
    rospy.sleep(2)
    publish_board(board_pub, move_8_castling3)
    rospy.sleep(2)
    publish_end_turn(end_turn_pub)
    rospy.sleep(2)

    """
    rospy.spin()


def publish_board(pub, board_state):
    # Flatten the 8x8 matrix into a list
    flattened_board = [square for row in board_state for square in row]

    # Create a ROS message and publish the board state
    board_msg = Int32MultiArray(data=flattened_board)
    pub.publish(board_msg)


"""
def publish_end_turn(pub):
    # Publish the end turn signal
    end_turn_msg = "end"
    pub.publish(end_turn_msg) """


if __name__ == "__main__":
    try:
        simulate_moves()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import chess

# Create a chess board using the python-chess library
chess_board = chess.Board()

# Publisher for move validation
move_validation_pub = rospy.Publisher("movevalidation", String, queue_size=10)


def move_callback(msg):
    global chess_board

    uci_move = msg.data
    rospy.loginfo(f"Received move: {uci_move}")

    try:
        move = chess.Move.from_uci(uci_move)

        # Check if the move is legal
        if move in chess_board.legal_moves:
            chess_board.push(move)
            rospy.loginfo(f"Move {uci_move} is valid.")
            move_validation_pub.publish("valid")
        else:
            rospy.logerr(f"Move {uci_move} is invalid.")
            move_validation_pub.publish("invalid")
    except ValueError:
        rospy.logerr(f"Invalid UCI format: {uci_move}")
        move_validation_pub.publish("invalid")

    rospy.loginfo(chess_board)


def main():
    rospy.init_node("chess_move_validator", anonymous=True)

    # Subscribe to the boardmove topic to receive UCI moves
    rospy.Subscriber("boardmove", String, move_callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

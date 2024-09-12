#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random
import time

# Publisher to publish player moves
move_pub = rospy.Publisher("boardmove", String, queue_size=10)
player_color_pub = rospy.Publisher("playercolor", String, queue_size=10)
difficulty_pub = rospy.Publisher("difficulty", String, queue_size=10)
surrender_pub = rospy.Publisher("ff", String, queue_size=10)


# Callback for move validation messages
def move_validation_callback(msg):
    rospy.loginfo(f"Move validation result: {msg.data}")


# Callback for bot moves (Stockfish's move)
def bot_move_callback(msg):
    rospy.loginfo(f"Bot move received: {msg.data}")
    # Simulate applying the move on the board by publishing it back to the boardmove topic
    time.sleep(2)  # Simulate delay for detecting the bot's move on the actual board
    move_pub.publish(msg.data)  # Publish bot's move to the `boardmove` topic


# Callback for end game status
def endgame_callback(msg):
    rospy.loginfo(f"Endgame status: {msg.data}")


def test_player_move():
    # Publish a move from the player
    moves = ["e2e4", "d2d4", "g1f3"]  # Example moves
    player_move = random.choice(moves)
    rospy.loginfo(f"Player move: {player_move}")
    move_pub.publish(player_move)


def test_bot_move():
    rospy.loginfo("Waiting for bot move...")


def test_set_player_color():
    colors = ["white", "black", "random"]
    selected_color = "white"
    rospy.loginfo(f"Setting player color to: {selected_color}")
    player_color_pub.publish(selected_color)


def test_set_difficulty():
    difficulty_level = random.randint(0, 20)
    rospy.loginfo(f"Setting Stockfish difficulty to: {difficulty_level}")
    difficulty_pub.publish(str(difficulty_level))


def test_player_surrender():
    rospy.loginfo("Player surrendering...")
    surrender_pub.publish("ff")


def main():
    rospy.init_node("chess_game_tester", anonymous=True)

    # Subscribe to validation, bot move, and endgame status topics
    rospy.Subscriber("movevalidation", String, move_validation_callback)
    rospy.Subscriber("botmoves", String, bot_move_callback)
    rospy.Subscriber("endgame", String, endgame_callback)

    # Allow some time for nodes to set up
    rospy.sleep(2)

    # Test setting the player's color
    test_set_player_color()
    rospy.sleep(2)

    # Test setting Stockfish's difficulty level
    test_set_difficulty()
    rospy.sleep(2)

    # Simulate a player move
    test_player_move()
    rospy.sleep(5)

    # Wait for bot's move
    test_bot_move()
    rospy.sleep(5)

    #

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

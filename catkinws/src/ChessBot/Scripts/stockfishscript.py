#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import chess
import chess.engine
import random

# Create a chess board using the python-chess library
chess_board = chess.Board()

# Publishers
move_validation_pub = rospy.Publisher("movevalidation", String, queue_size=10)
bot_move_pub = rospy.Publisher("botmoves", String, queue_size=10)
endgame_pub = rospy.Publisher(
    "endgame", String, queue_size=10
)  # New publisher for endgame status
gamestatus_pub = rospy.Publisher("gamestatus", String, queue_size=10)
endturn_pub = rospy.Publisher("end_turn", String, queue_size=10)
# Stockfish engine setup
engine_path = (
    "/home/speed/stockfish/Stockfish/src/stockfish"  # Change this to the correct path
)
engine = chess.engine.SimpleEngine.popen_uci(engine_path)

# Variables for managing player color, game state, difficulty, and bot move confirmation
player_color = None  # Will be 'white' or 'black'
game_over = False
stockfish_difficulty = 10  # Default difficulty (mid-level)
bot_move_confirmed = False  # Tracks whether the bot move was confirmed


# Reset the game state for a new game
def reset_game():
    global chess_board, player_color, game_over, stockfish_difficulty

    chess_board.reset()  # Reset the chessboard
    player_color = None  # Reset player color
    game_over = False  # Reset game status
    stockfish_difficulty = 10  # Reset Stockfish difficulty to default
    rospy.loginfo("Game has been reset. Waiting for new configuration.")


def move_callback(msg):
    global chess_board, game_over

    if game_over:
        rospy.logwarn("Game is already over.")
        return

    uci_move = msg.data
    rospy.loginfo(f"Received move: {uci_move}")

    try:
        move = chess.Move.from_uci(uci_move)

        # Check if the move is legal
        if move in chess_board.legal_moves:
            chess_board.push(move)
            rospy.loginfo(f"Move {uci_move} is valid.")
            move_validation_pub.publish("valid")

            # Check if the game is over after this move
            check_game_over()

            rospy.loginfo(chess_board)
            game_status()
            # If the game is not over, and it's the bot's turn, trigger the bot move
            if not game_over and chess_board.turn == (
                chess.BLACK if player_color == "white" else chess.WHITE
            ):
                make_bot_move()
        else:
            rospy.logerr(f"Move {uci_move} is invalid.")
            move_validation_pub.publish("invalid")
    except ValueError:
        rospy.logerr(f"Invalid UCI format: {uci_move}")
        move_validation_pub.publish("invalid")


def game_status():
    global chess_board, game_over

    if chess_board.is_check():
        rospy.loginfo("Check!")
        if chess_board.turn == chess.WHITE:
            gamestatus_pub.publish("White Check!")
            rospy.loginfo("White is in check.")
        else:
            gamestatus_pub.publish("Black Check!")
            rospy.loginfo("Black is in check.")
    else:
        if chess_board.turn == chess.WHITE:
            gamestatus_pub.publish("White's turn")
            rospy.loginfo("White's turn")
        else:
            gamestatus_pub.publish("Black's turn")
            rospy.loginfo("Black's turn")


def player_color_callback(msg):
    global player_color

    color = msg.data.lower()
    if color == "random":
        player_color = random.choice(["white", "black"])
    elif color in ["white", "black"]:
        player_color = color
    else:
        rospy.logerr(f"Invalid player color: {color}")
        return
    if player_color == "white":
        gamestatus_pub.publish("Your Playing as White")
    else:
        gamestatus_pub.publish("Your Playing as Black")
    # gamestatus_pub.publish(player_color)
    rospy.loginfo(f"Player color set to: {player_color}")


def surrender_callback(msg):
    global game_over

    if msg.data == "ff":
        rospy.loginfo("Player has surrendered. Game over.")
        game_over = True
        result_message = (
            f"{'White' if player_color == 'black' else 'Black'} wins by resignation."
        )
        rospy.loginfo(result_message)
        endgame_pub.publish(result_message)


def difficulty_callback(msg):
    global stockfish_difficulty

    try:
        level = int(msg.data)
        if 0 <= level <= 20:
            stockfish_difficulty = level
            rospy.loginfo(f"Stockfish difficulty set to {stockfish_difficulty}.")
        else:
            rospy.logerr(
                f"Invalid difficulty level: {level}. Must be between 0 and 20."
            )
    except ValueError:
        rospy.logerr(
            f"Invalid difficulty input: {msg.data}. Must be an integer between 0 and 20."
        )


# Callback to reset the game when a new game message is received
def new_game_callback(msg):
    if msg.data.lower() == "newgame":
        reset_game()
        # endgame_pub.publish("newgame")  # Publish a message to indicate a new game


# Callback to handle bot move confirmation
def bot_move_status_callback(msg):
    global bot_move_confirmed
    if msg.data == "confirmed":
        rospy.loginfo("Bot move confirmed.")
        bot_move_confirmed = True
    else:
        rospy.logerr(f"Unexpected confirmation message: {msg.data}")


def make_bot_move():
    global chess_board, engine, stockfish_difficulty, bot_move_confirmed

    # Set the Stockfish difficulty (skill level)
    engine.configure({"Skill Level": stockfish_difficulty})

    result = engine.play(
        chess_board, chess.engine.Limit(time=2.0)
    )  # Bot move with 2 seconds of analysis
    bot_move = result.move
    is_capture = chess_board.is_capture(bot_move)
    rospy.loginfo(f"Bot move: {bot_move.uci()} (Capture: {is_capture})")
    bot_move_pub.publish(
        f"{bot_move.uci()} {is_capture}"
    )  # Publish the move, but don't apply it to the board yet

    # Wait for external confirmation of the bot's move
    bot_move_confirmed = False
    rospy.loginfo("Waiting for external confirmation of the bot's move...")

    # Loop until confirmation is received
    while not bot_move_confirmed:
        rospy.sleep(0.1)  # Small delay to prevent busy-waiting
    endturn_pub.publish("endturn")
    # After confirmation, check if the game is over
    check_game_over()


def check_game_over():
    global chess_board, game_over

    if chess_board.is_checkmate():
        rospy.loginfo("Checkmate! Game over.")
        game_over = True
        winner = "White" if chess_board.turn == chess.BLACK else "Black"
        result_message = f"{winner} wins by checkmate."
        endgame_pub.publish(result_message)
    elif chess_board.is_stalemate():
        rospy.loginfo("Stalemate! Game over.")
        game_over = True
        result_message = "Game is drawn by stalemate."
        endgame_pub.publish(result_message)
    elif chess_board.is_insufficient_material():
        rospy.loginfo("Insufficient material! Game over.")
        game_over = True
        result_message = "Game is drawn by insufficient material."
        endgame_pub.publish(result_message)
    elif chess_board.is_seventyfive_moves():
        rospy.loginfo("75-move rule! Game over.")
        game_over = True
        result_message = "Game is drawn by the 75-move rule."
        endgame_pub.publish(result_message)
    elif chess_board.is_fivefold_repetition():
        rospy.loginfo("Fivefold repetition! Game over.")
        game_over = True
        result_message = "Game is drawn by fivefold repetition."
        endgame_pub.publish(result_message)
    elif chess_board.is_variant_draw():
        rospy.loginfo("Variant draw! Game over.")
        game_over = True
        result_message = "Game is drawn."
        endgame_pub.publish(result_message)


def main():
    rospy.init_node("chess_game_with_stockfish", anonymous=True)

    # Subscribe to the boardmove topic to receive UCI moves from the player and external script
    rospy.Subscriber("boardmove", String, move_callback)

    # Subscribe to the playercolor topic to determine the player's color
    rospy.Subscriber("playercolor", String, player_color_callback)

    # Subscribe to the ff topic to detect player resignation (surrender)
    rospy.Subscriber("ff", String, surrender_callback)

    # Subscribe to the difficulty topic to set Stockfish's difficulty
    rospy.Subscriber("difficulty", String, difficulty_callback)

    # Subscribe to the newgame topic to reset the game
    rospy.Subscriber("newgame", String, new_game_callback)

    # Subscribe to botmovestatus topic to receive confirmation after the bot makes a move
    rospy.Subscriber("botmovestatus", String, bot_move_status_callback)

    # Keep the node running
    rospy.spin()

    # Close the engine at the end
    engine.quit()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

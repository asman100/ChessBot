#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Bool, String

# Global variables to track game state
saved_board_state = [[0 for _ in range(8)] for _ in range(8)]
piece_board = [
    ["r", "n", "b", "q", "k", "b", "n", "r"],  # Black pieces (row 8)
    ["p", "p", "p", "p", "p", "p", "p", "p"],  # Black pawns (row 7)
    [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 6
    [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 5
    [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 4
    [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 3
    ["P", "P", "P", "P", "P", "P", "P", "P"],  # White pawns (row 2)
    ["R", "N", "B", "Q", "K", "B", "N", "R"],  # White pieces (row 1)
]

old_piece_board = [row[:] for row in piece_board]  # Old state of the board to compare
temp_old_piece_board = [row[:] for row in piece_board]
castling_rights = {
    "wK": True,
    "wQ": True,
    "bK": True,
    "bQ": True,
}
removed_white_pieces = []
removed_white_positions = []
removed_black_pieces = []
removed_black_positions = []
current_turn = "w"  # 'w' for white's turn, 'b' for black's turn
uci_move = None
move_validated = None  # None to indicate awaiting validation result

# Publisher for UCI move
move_pub = rospy.Publisher("boardmove", String, queue_size=10)
pieceboard_pub = rospy.Publisher("pieceboard", Int32MultiArray, queue_size=10)

game_started = False  # Track whether the game has started


def reset_game_state():
    global saved_board_state, piece_board, old_piece_board, temp_old_piece_board
    global castling_rights, removed_white_pieces, removed_white_positions
    global removed_black_pieces, removed_black_positions, current_turn, move_validated, uci_move, game_started
    game_started = False

    saved_board_state = [[0 for _ in range(8)] for _ in range(8)]
    piece_board = [
        ["r", "n", "b", "q", "k", "b", "n", "r"],  # Black pieces (row 8)
        ["p", "p", "p", "p", "p", "p", "p", "p"],  # Black pawns (row 7)
        [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 6
        [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 5
        [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 4
        [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 3
        ["P", "P", "P", "P", "P", "P", "P", "P"],  # White pawns (row 2)
        ["R", "N", "B", "Q", "K", "B", "N", "R"],  # White pieces (row 1)
    ]
    old_piece_board = [row[:] for row in piece_board]
    temp_old_piece_board = [row[:] for row in piece_board]

    castling_rights = {
        "wK": True,
        "wQ": True,
        "bK": True,
        "bQ": True,
    }

    removed_white_pieces = []
    removed_white_positions = []
    removed_black_pieces = []
    removed_black_positions = []

    current_turn = "w"
    move_validated = None
    uci_move = None

    rospy.loginfo("Game state has been reset.")


def convert_piece_board_to_array(piece_board):
    piece_map = {
        "r": -6,
        "n": -5,
        "b": -4,
        "q": -3,
        "k": -2,
        "p": -1,  # Black pieces
        "R": 6,
        "N": 5,
        "B": 4,
        "Q": 3,
        "K": 2,
        "P": 1,  # White pieces
        " ": 0,  # Empty squares
    }
    flat_board = []
    for row in piece_board:
        for piece in row:
            flat_board.append(piece_map.get(piece, 0))
    return flat_board


def publish_piece_board():
    global piece_board
    board_array = convert_piece_board_to_array(piece_board)
    msg = Int32MultiArray()
    msg.data = board_array
    pieceboard_pub.publish(msg)
    rospy.loginfo("Piece board published.")


def startgame_callback(msg):
    global game_started
    if msg.data.lower() == "start":
        game_started = True
        rospy.loginfo("Game started!")
    else:
        rospy.logwarn("Invalid start game message.")


# Callback for move validation
def move_validation_callback(msg):
    global move_validated
    if msg.data == "valid":
        rospy.loginfo("Move is valid!")
        move_validated = "valid"
    elif msg.data == "invalid":
        rospy.loginfo("Move is invalid!")
        move_validated = "invalid"


def end_turn_callback(data):

    global old_piece_board, piece_board, current_turn, uci_move, move_validated, temp_old_piece_board, game_started

    if not game_started:
        rospy.logwarn("Game has not started yet!")
        return
    print_board(piece_board)

    if data.data:
        # Store temporary copies of old_piece_board and piece_board
        temp_old_piece_board = [row[:] for row in old_piece_board]
        temp_piece_board = [row[:] for row in piece_board]

        # Detect castling
        uci_move = detect_castling()

        # If castling is detected, process the castling move
        if uci_move:
            rospy.loginfo(f"Castling move detected: {uci_move}")
        else:
            # Detect and validate a normal move
            uci_move = detect_move_uci()
            old_piece_board = [row[:] for row in piece_board]

        # Publish the detected UCI move
        if uci_move:
            rospy.loginfo(f"Published move: {uci_move}")
            move_validated = None
            move_pub.publish(uci_move)

            rospy.loginfo("Waiting for move validation...")

            # Wait for validation reply (either "valid" or "invalid")
            while move_validated is None:
                rospy.sleep(0.01)
            if move_validated == "valid":
                rospy.loginfo(f"Move {uci_move} was valid!")
                # No need to restore piece_board; the move is valid
                current_turn = "b" if current_turn == "w" else "w"
                rospy.loginfo(f"Current turn: {current_turn}")

            elif move_validated == "invalid":
                rospy.logwarn(f"Move {uci_move} was invalid!")
                # Restore piece_board to its previous state
                piece_board = [row[:] for row in temp_piece_board]
                old_piece_board = [row[:] for row in temp_old_piece_board]
        else:
            rospy.logwarn("No valid move detected!")


def detect_move_uci():
    global old_piece_board, piece_board

    start_pos = None
    end_pos = None

    # Compare old and current board states to detect the move
    for row in range(8):
        for col in range(8):
            # Piece removed (start position)
            if old_piece_board[row][col] != " " and piece_board[row][col] == " ":
                start_pos = (row, col)
            # Piece placed (end position)
            elif old_piece_board[row][col] == " " and piece_board[row][col] != " ":
                end_pos = (row, col)

    # Detect castling explicitly using both the king's and rook's movements
    # White kingside castling (e1 to g1 and rook moves from h1 to f1)
    if (
        start_pos == (7, 4)
        and end_pos == (7, 6)
        and piece_board[7][5] == "R"
        and piece_board[7][6] == "K"
    ):
        rospy.loginfo("White kingside castling detected")
        return "e1g1"

    # White queenside castling (e1 to c1 and rook moves from a1 to d1)
    elif (
        start_pos == (7, 4)
        and end_pos == (7, 2)
        and piece_board[7][3] == "R"
        and piece_board[7][2] == "K"
    ):
        rospy.loginfo("White queenside castling detected")
        return "e1c1"

    # Black kingside castling (e8 to g8 and rook moves from h8 to f8)
    elif (
        start_pos == (0, 4)
        and end_pos == (0, 6)
        and piece_board[0][5] == "r"
        and piece_board[0][6] == "k"
    ):
        rospy.loginfo("Black kingside castling detected")
        return "e8g8"

    # Black queenside castling (e8 to c8 and rook moves from a8 to d8)
    elif (
        start_pos == (0, 4)
        and end_pos == (0, 2)
        and piece_board[0][3] == "r"
        and piece_board[0][2] == "k"
    ):
        rospy.loginfo("Black queenside castling detected")
        return "e8c8"

    # Default behavior for normal moves
    if start_pos and end_pos:
        return convert_to_uci(start_pos, end_pos)

    rospy.logwarn("Move detection failed: start or end position not found.")
    return None


def convert_to_uci(start_pos, end_pos):
    """Convert row, col positions to UCI format (e.g., e2e4)."""
    rows = "87654321"
    cols = "abcdefgh"
    start_square = cols[start_pos[1]] + rows[start_pos[0]]
    end_square = cols[end_pos[1]] + rows[end_pos[0]]
    return start_square + end_square


def detect_castling():
    global castling_rights, piece_board, old_piece_board, current_turn

    # Check for White castling
    if current_turn == "w":
        # Kingside castling: King moves from e1 to g1 and rook moves from h1 to f1
        if (
            old_piece_board[7][4] == "K"
            and old_piece_board[7][7] == "R"
            and piece_board[7][4] == " "
            and piece_board[7][6] == "K"
            and piece_board[7][5] == "R"
        ):
            if castling_rights["wK"]:
                rospy.loginfo("White kingside castling detected")
                castling_rights["wK"], castling_rights["wQ"] = False, False
                return "e1g1"

        # Queenside castling: King moves from e1 to c1 and rook moves from a1 to d1
        if (
            old_piece_board[7][4] == "K"
            and old_piece_board[7][0] == "R"
            and piece_board[7][4] == " "
            and piece_board[7][2] == "K"
            and piece_board[7][3] == "R"
        ):
            if castling_rights["wQ"]:
                rospy.loginfo("White queenside castling detected")
                castling_rights["wK"], castling_rights["wQ"] = False, False
                return "e1c1"

    # Check for Black castling
    elif current_turn == "b":
        # Kingside castling: King moves from e8 to g8 and rook moves from h8 to f8
        if (
            old_piece_board[0][4] == "k"
            and old_piece_board[0][7] == "r"
            and piece_board[0][4] == " "
            and piece_board[0][6] == "k"
            and piece_board[0][5] == "r"
        ):
            if castling_rights["bK"]:
                rospy.loginfo("Black kingside castling detected")
                castling_rights["bK"], castling_rights["bQ"] = False, False
                return "e8g8"

        # Queenside castling: King moves from e8 to c8 and rook moves from a8 to d8
        if (
            old_piece_board[0][4] == "k"
            and old_piece_board[0][0] == "r"
            and piece_board[0][4] == " "
            and piece_board[0][2] == "k"
            and piece_board[0][3] == "r"
        ):
            if castling_rights["bQ"]:
                rospy.loginfo("Black queenside castling detected")
                castling_rights["bK"], castling_rights["bQ"] = False, False
                return "e8c8"

    return None


def chessboard_callback(data):
    global saved_board_state, piece_board
    global removed_white_pieces, removed_white_positions
    global removed_black_pieces, removed_black_positions
    global game_started
    sensor_array = data.data
    new_board_state = [sensor_array[i : i + 8] for i in range(0, 64, 8)]
    rospy.loginfo("New board state detected:")
    print_boardstate(new_board_state)
    if not game_started:
        rospy.logwarn("Game has not started yet!")
        # Initialize the starting positions
        initial_piece_board = [
            ["r", "n", "b", "q", "k", "b", "n", "r"],  # Black pieces (row 8)
            ["p", "p", "p", "p", "p", "p", "p", "p"],  # Black pawns (row 7)
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 6
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 5
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 4
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 3
            ["P", "P", "P", "P", "P", "P", "P", "P"],  # White pawns (row 2)
            ["R", "N", "B", "Q", "K", "B", "N", "R"],  # White pieces (row 1)
        ]

        piece_board = []
        for row in range(8):
            piece_row = []
            for col in range(8):
                if new_board_state[row][col] == 1:
                    # Piece is present at this location
                    piece_row.append(initial_piece_board[row][col])
                else:
                    # No piece present at this location
                    piece_row.append(" ")
            piece_board.append(piece_row)
        # Publish the piece_board
        publish_piece_board()
        # Update saved_board_state
        saved_board_state = new_board_state
        print_board(piece_board)
        return  # Exit the function since the game hasn't started yet

    for row in range(8):
        for col in range(8):
            # Piece removed
            if saved_board_state[row][col] == 1 and new_board_state[row][col] == 0:
                removed_piece = piece_board[row][col]
                if removed_piece != " ":
                    if removed_piece.isupper():
                        removed_white_pieces.append(removed_piece)
                        removed_white_positions.append((row, col))
                        rospy.loginfo(
                            f"White piece {removed_piece} removed from {(row, col)}"
                        )
                    else:
                        removed_black_pieces.append(removed_piece)
                        removed_black_positions.append((row, col))
                        rospy.loginfo(
                            f"Black piece {removed_piece} removed from {(row, col)}"
                        )
                    piece_board[row][col] = " "

            # Piece placed
            elif saved_board_state[row][col] == 0 and new_board_state[row][col] == 1:
                if current_turn == "w" and removed_white_pieces:
                    # Check for castling
                    castling_piece = detect_castling()  # No arguments passed
                    if castling_piece == "K":
                        # Place the king
                        piece_board[row][col] = "K"
                        # Remove king and rook from removed pieces
                        king_index = removed_white_pieces.index("K")
                        rook_index = removed_white_pieces.index("R")
                        # Remove king
                        removed_white_pieces.pop(king_index)
                        removed_white_positions.pop(king_index)
                        # Adjust index if king was before rook
                        if king_index < rook_index:
                            rook_index -= 1
                        # Remove rook
                        removed_white_pieces.pop(rook_index)
                        removed_white_positions.pop(rook_index)
                    else:
                        # Normal move
                        piece = removed_white_pieces.pop(0)
                        piece_board[row][col] = piece

                elif current_turn == "b" and removed_black_pieces:
                    # Check for castling
                    castling_piece = detect_castling()  # No arguments passed
                    if castling_piece == "k":
                        # Place the king
                        piece_board[row][col] = "k"
                        # Remove king and rook from removed pieces
                        king_index = removed_black_pieces.index("k")
                        rook_index = removed_black_pieces.index("r")
                        # Remove king
                        removed_black_pieces.pop(king_index)
                        removed_black_positions.pop(king_index)
                        # Adjust index if king was before rook
                        if king_index < rook_index:
                            rook_index -= 1
                        # Remove rook
                        removed_black_pieces.pop(rook_index)
                        removed_black_positions.pop(rook_index)
                    else:
                        # Normal move
                        piece = removed_black_pieces.pop(0)
                        piece_board[row][col] = piece
    publish_piece_board()
    saved_board_state = new_board_state


def endgame_callback(msg):
    # Any message on the endgame topic triggers a reset of the game state
    rospy.loginfo("Endgame detected. Resetting game state.")
    reset_game_state()


def print_boardstate(board):
    for row in board:
        print(row)
    print("\n")


def print_board(board):
    for row in board:
        print(" ".join(row))
    print("\n")


def checkboard_callback(data):
    publish_piece_board()


def main():

    rospy.init_node("chessboard_move_detector", anonymous=True)

    # Subscribe to chessboard updates
    rospy.Subscriber("chessboard", Int32MultiArray, chessboard_callback)

    # Subscribe to end turn topic to switch turns
    rospy.Subscriber("end_turn", String, end_turn_callback)

    # Subscribe to move validation
    rospy.Subscriber("movevalidation", String, move_validation_callback)
    # Subscribe to the startgame topic to trigger the game start
    rospy.Subscriber("startgame", String, startgame_callback)
    # Subscribe to the endgame topic to reset the game when it's over
    rospy.Subscriber("endgame", String, endgame_callback)

    rospy.Subscriber("checkpos", String, checkboard_callback)

    rospy.Rate(60)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

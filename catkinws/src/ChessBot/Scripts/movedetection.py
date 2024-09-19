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


def detect_castling(removed_positions, placed_positions, piece_positions):
    # Castling involves moving the king and rook simultaneously
    # Define starting and ending positions for kings and rooks
    # White King Side Castling
    white_king_start = (7, 4)
    white_rook_king_side_start = (7, 7)
    white_king_end_king_side = (7, 6)
    white_rook_end_king_side = (7, 5)

    # White Queen Side Castling
    white_rook_queen_side_start = (7, 0)
    white_king_end_queen_side = (7, 2)
    white_rook_end_queen_side = (7, 3)

    # Black King Side Castling
    black_king_start = (0, 4)
    black_rook_king_side_start = (0, 7)
    black_king_end_king_side = (0, 6)
    black_rook_end_king_side = (0, 5)

    # Black Queen Side Castling
    black_rook_queen_side_start = (0, 0)
    black_king_end_queen_side = (0, 2)
    black_rook_end_queen_side = (0, 3)

    # Check for white castling
    if (
        white_king_start in removed_positions
        and white_rook_king_side_start in removed_positions
        and (
            white_king_end_king_side in placed_positions
            and white_rook_end_king_side in placed_positions
        )
    ):
        # White king side castling
        return True
    if (
        white_king_start in removed_positions
        and white_rook_queen_side_start in removed_positions
        and (
            white_king_end_queen_side in placed_positions
            and white_rook_end_queen_side in placed_positions
        )
    ):
        # White queen side castling
        return True

    # Check for black castling
    if (
        black_king_start in removed_positions
        and black_rook_king_side_start in removed_positions
        and (
            black_king_end_king_side in placed_positions
            and black_rook_end_king_side in placed_positions
        )
    ):
        # Black king side castling
        return True
    if (
        black_king_start in removed_positions
        and black_rook_queen_side_start in removed_positions
        and (
            black_king_end_queen_side in placed_positions
            and black_rook_end_queen_side in placed_positions
        )
    ):
        # Black queen side castling
        return True

    return False  # Not a castling move


def pos_to_square(pos):
    row, col = pos
    file_map = {0: "a", 1: "b", 2: "c", 3: "d", 4: "e", 5: "f", 6: "g", 7: "h"}
    rank_map = {0: "8", 1: "7", 2: "6", 3: "5", 4: "4", 5: "3", 6: "2", 7: "1"}
    return f"{file_map[col]}{rank_map[row]}"


def print_board(piece_positions):
    board = [[" " for _ in range(8)] for _ in range(8)]
    for pos, piece in piece_positions.items():
        row, col = pos
        board[row][col] = piece
    print("Current Board:")
    for row in board:
        print(" ".join(row))


def chessboard_callback(data):
    global saved_board_state, piece_positions, game_started, initial_piece_board, piece_board

    sensor_array = data.data
    new_board_state_flat = sensor_array
    new_board_state = [new_board_state_flat[i : i + 8] for i in range(0, 64, 8)]

    rospy.loginfo("New board state detected:")
    print_boardstate(new_board_state)

    if not game_started:
        rospy.logwarn("Game has not started yet!")
        # Initialize the starting positions
        initial_piece_board = [
            ["r", "n", "b", "q", "k", "b", "n", "r"],  # Black pieces (row 0)
            ["p", "p", "p", "p", "p", "p", "p", "p"],  # Black pawns (row 1)
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 2
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 3
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 4
            [" ", " ", " ", " ", " ", " ", " ", " "],  # Empty row 5
            ["P", "P", "P", "P", "P", "P", "P", "P"],  # White pawns (row 6)
            ["R", "N", "B", "Q", "K", "B", "N", "R"],  # White pieces (row 7)
        ]
        piece_positions = {}
        for row in range(8):
            for col in range(8):
                if new_board_state[row][col] == 1:
                    # Piece is present at this location
                    piece = initial_piece_board[row][col]
                    if piece != " ":
                        piece_positions[(row, col)] = piece
        # Update saved_board_state
        saved_board_state = new_board_state
        print_board(piece_positions)
        return  # Exit the function since the game hasn't started yet

    # Lists to keep track of changes
    removed_positions = []
    placed_positions = []

    for row in range(8):
        for col in range(8):
            if saved_board_state[row][col] != new_board_state[row][col]:
                if saved_board_state[row][col] == 1 and new_board_state[row][col] == 0:
                    # Piece was removed from (row, col)
                    removed_positions.append((row, col))
                elif (
                    saved_board_state[row][col] == 0 and new_board_state[row][col] == 1
                ):
                    # Piece was placed at (row, col)
                    placed_positions.append((row, col))

    # Process the changes
    if len(removed_positions) == 1 and len(placed_positions) == 1:
        # Normal move
        from_pos = removed_positions[0]
        to_pos = placed_positions[0]
        moved_piece = piece_positions.get(from_pos)
        if moved_piece:
            # Update piece_positions
            del piece_positions[from_pos]
            piece_positions[to_pos] = moved_piece
            rospy.loginfo(
                f"Piece {moved_piece} moved from {pos_to_square(from_pos)} to {pos_to_square(to_pos)}"
            )
        else:
            rospy.logwarn("Moved piece not found in piece_positions")
    elif len(removed_positions) == 2 and len(placed_positions) == 2:
        # Possible castling move
        if detect_castling(removed_positions, placed_positions, piece_positions):
            rospy.loginfo("Castling move detected")
            # Update piece_positions accordingly
            for from_pos in removed_positions:
                moved_piece = piece_positions.get(from_pos)
                if moved_piece:
                    # Find corresponding to_pos
                    to_pos = None
                    for pos in placed_positions:
                        if pos not in piece_positions:
                            to_pos = pos
                            break
                    if to_pos:
                        del piece_positions[from_pos]
                        piece_positions[to_pos] = moved_piece
                        rospy.loginfo(
                            f"Piece {moved_piece} moved from {pos_to_square(from_pos)} to {pos_to_square(to_pos)}"
                        )
                    else:
                        rospy.logwarn("Matching placed position not found for castling")
                else:
                    rospy.logwarn(
                        "Moved piece not found in piece_positions for castling"
                    )
        else:
            # Capture move
            attacker_pos = removed_positions[0]
            captured_pos = removed_positions[1]
            to_pos = placed_positions[0]
            if to_pos == captured_pos or to_pos == placed_positions[1]:
                moved_piece = piece_positions.get(attacker_pos)
                captured_piece = piece_positions.get(captured_pos)
                if moved_piece and captured_piece:
                    # Update piece_positions
                    del piece_positions[attacker_pos]
                    del piece_positions[captured_pos]
                    piece_positions[to_pos] = moved_piece
                    rospy.loginfo(
                        f"Piece {moved_piece} moved from {pos_to_square(attacker_pos)} to {pos_to_square(to_pos)}, capturing {captured_piece}"
                    )
                else:
                    rospy.logwarn(
                        "Moved or captured piece not found in piece_positions"
                    )
            else:
                rospy.logwarn("Complex move detected, unable to resolve")
    elif len(removed_positions) == 2 and len(placed_positions) == 1:
        # Capture move where two pieces removed and one placed
        attacker_pos = removed_positions[0]
        captured_pos = removed_positions[1]
        to_pos = placed_positions[0]
        if to_pos == captured_pos:
            moved_piece = piece_positions.get(attacker_pos)
            captured_piece = piece_positions.get(captured_pos)
            if moved_piece and captured_piece:
                # Update piece_positions
                del piece_positions[attacker_pos]
                del piece_positions[captured_pos]
                piece_positions[to_pos] = moved_piece
                rospy.loginfo(
                    f"Piece {moved_piece} moved from {pos_to_square(attacker_pos)} to {pos_to_square(to_pos)}, capturing {captured_piece}"
                )
            else:
                rospy.logwarn("Moved or captured piece not found in piece_positions")
        else:
            rospy.logwarn("Complex move detected, unable to resolve")
    elif len(removed_positions) == 0 and len(placed_positions) == 1:
        # Piece added to the board (e.g., promotion)
        to_pos = placed_positions[0]
        # You may need additional logic to determine which piece was added
        rospy.loginfo(f"A piece was placed at {pos_to_square(to_pos)} from outside")
        # For example, you can prompt the user or have a predefined piece
    elif len(removed_positions) == 1 and len(placed_positions) == 0:
        # Piece removed from the board (e.g., capture off the board)
        from_pos = removed_positions[0]
        removed_piece = piece_positions.get(from_pos)
        if removed_piece:
            del piece_positions[from_pos]
            rospy.loginfo(
                f"Piece {removed_piece} was removed from {pos_to_square(from_pos)}"
            )
        else:
            rospy.logwarn("Removed piece not found in piece_positions")
    else:
        rospy.logwarn("Complex move detected, unable to resolve")
        # Handle special cases like en passant or multiple moves

    # Update saved_board_state
    saved_board_state = new_board_state
    piece_board = reconstruct_piece_board(piece_positions)
    print_board(piece_positions)


def endgame_callback(msg):
    # Any message on the endgame topic triggers a reset of the game state
    rospy.loginfo("Endgame detected. Resetting game state.")
    reset_game_state()


def print_boardstate(board):
    for row in board:
        print(row)
    print("\n")


def reconstruct_piece_board(piece_positions):
    # Create an empty board
    piece_board = [[" " for _ in range(8)] for _ in range(8)]
    for pos, piece in piece_positions.items():
        row, col = pos
        piece_board[row][col] = piece
    return piece_board


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


def publish_piece_board(piece_board):
    global pieceboard_pub
    board_array = convert_piece_board_to_array(piece_board)
    msg = Int32MultiArray()
    msg.data = board_array
    pieceboard_pub.publish(msg)
    rospy.loginfo("Piece board published.")


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

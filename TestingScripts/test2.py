import chess
import chess.engine

move_number = 1


def main():
    # Initialize the chess board
    board = chess.Board()
    move_number = 1

    # Specify the path to your Stockfish engine
    engine_path = (
        "/home/thabet/Stockfish/src/stockfish"  # Replace with your actual path
    )
    engine = chess.engine.SimpleEngine.popen_uci(engine_path)

    move_number = 1  # Keep track of the move number

    # Play the game until it is over
    while not board.is_game_over():
        # Let Stockfish choose the best move
        result = engine.play(board, chess.engine.Limit(time=0.1))
        move = result.move

        # Check if the move is a capture
        is_capture = board.is_capture(move)

        # Print the move in UCI format along with capture information (True/False)
        move_info = f"{move_number}. {move.uci()} {is_capture}"
        print(move_info)

        # Make the move on the board
        board.push(move)

        move_number += 1

    # Print the game result
    print("Game over:", board.result())

    # Close the engine
    engine.quit()


if __name__ == "__main__":
    main()

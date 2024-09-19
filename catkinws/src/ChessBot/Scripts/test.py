#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray


def main():
    rospy.init_node("chessboard_array_tester", anonymous=True)
    tester = ChessboardArrayTester()
    rospy.Subscriber("chessboard", Int32MultiArray, tester.chessboard_callback)
    rospy.sleep(1)  # Allow time for connection

    try:
        tester.run_test()
    except KeyboardInterrupt:
        pass


class ChessboardArrayTester:
    def __init__(self):
        self.data_received = False
        self.data = []
        self.new_data = False

    def chessboard_callback(self, msg):
        self.data = list(msg.data)
        self.data_received = True
        self.new_data = True

    def run_test(self):
        print("Chessboard Array Tester")
        print("-----------------------")
        print("This tool will help you determine how to access the sensor data array.")
        print(
            "Enter a chessboard position (e.g., 'e2') to see the data values using different access methods."
        )
        print("Type 'exit' at any time to quit.\n")

        while not rospy.is_shutdown():
            # Wait until we have data
            if not self.data_received:
                print("Waiting for data from 'chessboard' topic...")
                rospy.sleep(1)
                continue

            user_input = (
                input("Enter a chessboard position (e.g., 'e2') or 'exit' to quit: ")
                .strip()
                .lower()
            )

            if user_input == "exit":
                break

            if not self.validate_square(user_input):
                print("Invalid square. Please enter a valid square (a1 to h8).\n")
                continue

            square = user_input
            # Now, for that square, try different methods to compute the index
            file = square[0]
            rank = int(square[1])

            file_map = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "f": 5, "g": 6, "h": 7}
            file_index = file_map[file]
            rank_index = rank - 1  # ranks from 1 to 8

            # Now, try different methods to compute index
            methods = []

            # Method 1: data[rank_index * 8 + file_index]
            index1 = rank_index * 8 + file_index
            methods.append(("Method 1 (rank_index * 8 + file_index)", index1))

            # Method 2: data[file_index * 8 + rank_index]
            index2 = file_index * 8 + rank_index
            methods.append(("Method 2 (file_index * 8 + rank_index)", index2))

            # Method 3: data[(7 - rank_index) * 8 + file_index]
            index3 = (7 - rank_index) * 8 + file_index
            methods.append(("Method 3 ((7 - rank_index) * 8 + file_index)", index3))

            # Method 4: data[rank_index * 8 + (7 - file_index)]
            index4 = rank_index * 8 + (7 - file_index)
            methods.append(("Method 4 (rank_index *8 + (7 - file_index))", index4))

            # Method 5: data[(7 - rank_index) * 8 + (7 - file_index)]
            index5 = (7 - rank_index) * 8 + (7 - file_index)
            methods.append(
                ("Method 5 ((7 - rank_index) *8 + (7 - file_index))", index5)
            )

            # Method 6: data[file_index + rank_index *8]
            index6 = file_index + rank_index * 8
            methods.append(("Method 6 (file_index + rank_index * 8)", index6))

            # Method 7: data[file_index + (7 - rank_index) *8]
            index7 = file_index + (7 - rank_index) * 8
            methods.append(("Method 7 (file_index + (7 - rank_index) *8)", index7))

            # Method 8: data[(7 - file_index) + rank_index *8]
            index8 = (7 - file_index) + rank_index * 8
            methods.append(("Method 8 ((7 - file_index) + rank_index *8)", index8))

            # Method 9: data[(7 - file_index) + (7 - rank_index) *8]
            index9 = (7 - file_index) + (7 - rank_index) * 8
            methods.append(
                ("Method 9 ((7 - file_index) + (7 - rank_index) *8)", index9)
            )

            # Now, for each method, print the index and the data value
            for method_name, index in methods:
                if index < 0 or index >= len(self.data):
                    value = "Index out of range"
                else:
                    value = self.data[index]
                print(f"{method_name}: index = {index}, value = {value}")

            print("\n")

    def validate_square(self, square):
        if len(square) != 2:
            return False
        file = square[0]
        rank = square[1]
        return file in "abcdefgh" and rank in "12345678"


if __name__ == "__main__":
    main()

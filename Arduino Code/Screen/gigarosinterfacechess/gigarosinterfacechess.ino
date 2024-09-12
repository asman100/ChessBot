#include "DIALOG.h" /* emWin library includes Arduino_H7_Video and Arduino_GigaDisplayTouch library */
#include <ros.h>    // ROS library for Arduino
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

static WM_HWIN hLoadingScreen, hDifficultyScreen, hPlayingScreen, hEndgameScreen;  // Handles for the screens
static SLIDER_Handle hSlider;      // Slider handle
static BUTTON_Handle hWhiteButton, hBlackButton, hRandomButton, hConfirmButton, hEndTurnButton, hForfeitButton, hNewGameButton;
static TEXT_Handle hStatusText, hEndgameText, hbotdifficultytext;
static bool inLoadingScreen = false;  // Track whether we're in the loading screen
static char endgameMessage[100] = "Game Over";  // Variable to store endgame message

// ROS NodeHandle and Publishers
ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher newgame_pub("newgame", &msg);
ros::Publisher end_turn_pub("end_turn", &msg);
ros::Publisher endgame_pub("endgame", &msg);
ros::Publisher playercolor_pub("playercolor", &msg);
ros::Publisher difficulty_pub("difficulty", &msg);
ros::Publisher startgame_pub("startgame", &msg);
ros::Publisher checkpos_pub("checkpos", &msg);
static int sliderValue = 10;        // Variable to track slider value
static char selectedColor[10] = "Random";

// Global variable to store the current board state (8x8 array)
int32_t board[8][8] = {0};  // Initialize as empty
char pieceMap[8][8];  // For converted chess piece characters

// Subscription for pieceboard topic
void pieceboardCallback(const std_msgs::Int32MultiArray& board_msg);
ros::Subscriber<std_msgs::Int32MultiArray> pieceboard_sub("pieceboard", &pieceboardCallback);

// Subscription for gamestatus topic
std_msgs::String gameStatusMsg;
ros::Subscriber<std_msgs::String> gamestatus_sub("gamestatus", [](const std_msgs::String &status) {
    TEXT_SetText(hStatusText, status.data);  // Update the game status text
});

// Subscription for endgame topic
void endgameCallback(const std_msgs::String& endgame_msg) {
    strncpy(endgameMessage, endgame_msg.data, sizeof(endgameMessage) - 1);  // Copy the message
    WM_DeleteWindow(hPlayingScreen);  // Remove the playing screen
    hEndgameScreen = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbEndgameScreen, 0);  // Show the endgame screen
}
ros::Subscriber<std_msgs::String> endgame_sub("endgame", &endgameCallback);

// Function to convert board values to chess pieces
char convertToPiece(int value) {
    switch (value) {
        case -6: return 'r'; case -5: return 'n'; case -4: return 'b';
        case -3: return 'q'; case -2: return 'k'; case -1: return 'p';  // Black pieces
        case 6: return 'R'; case 5: return 'N'; case 4: return 'B';
        case 3: return 'Q'; case 2: return 'K'; case 1: return 'P';  // White pieces
        case 0: return ' ';  // Empty square
        default: return ' ';  // Default empty
    }
}

// Callback function for when a message is received on the "pieceboard" topic
void pieceboardCallback(const std_msgs::Int32MultiArray& board_msg) {
    int index = 0;
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            board[row][col] = board_msg.data[index];  // Store the raw board values
            pieceMap[row][col] = convertToPiece(board[row][col]);  // Convert to chess pieces
            index++;
        }
    }
    WM_InvalidateWindow(hPlayingScreen);  // Request redraw of the playing screen
}

/* Window handler for the endgame screen */
static void _cbEndgameScreen(WM_MESSAGE * pMsg) {
    switch (pMsg->MsgId) {
        case WM_CREATE: {
            // Display the endgame message
            hEndgameText = TEXT_CreateEx(100, 150, 600, 80, pMsg->hWin, WM_CF_SHOW, TEXT_CF_HCENTER | TEXT_CF_VCENTER, GUI_ID_TEXT0, endgameMessage);
            TEXT_SetFont(hEndgameText, &GUI_Font32B_ASCII);
            TEXT_SetTextColor(hEndgameText, GUI_WHITE);

            // Create the "New Game" button
            hNewGameButton = BUTTON_CreateEx(340, 300, 120, 50, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON0);
            BUTTON_SetText(hNewGameButton, "New Game");
            BUTTON_SetBkColor(hNewGameButton, BUTTON_CI_UNPRESSED, GUI_WHITE);
            BUTTON_SetBkColor(hNewGameButton, BUTTON_CI_PRESSED, GUI_LIGHTGRAY);
            BUTTON_SetTextColor(hNewGameButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            break;
        }

        case WM_PAINT:
            GUI_SetBkColor(GUI_BLACK);  // Set background to black
            GUI_Clear();  // Clear the screen
            break;

        case WM_NOTIFY_PARENT:
            if (pMsg->Data.v == WM_NOTIFICATION_RELEASED && WM_GetId(pMsg->hWinSrc) == GUI_ID_BUTTON0) {
                WM_DeleteWindow(pMsg->hWin);  // Delete the endgame screen
                WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbStartScreen, 0);  // Return to the start screen
            }
            break;

        default:
            WM_DefaultProc(pMsg);
    }
}

/* Window handler for the start screen */
static void _cbStartScreen(WM_MESSAGE * pMsg) {
    static BUTTON_Handle hStartButton;  // Button handle
    GUI_RECT rect;

    switch (pMsg->MsgId) {
        case WM_CREATE:
            hStartButton = BUTTON_CreateEx(340, 250, 120, 50, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON0);
            BUTTON_SetText(hStartButton, "Start");
            break;

        case WM_PAINT: {
            GUI_SetBkColor(0x03989e);  // Cyan background
            GUI_Clear();

            // Set white color for the filled box
            GUI_SetColor(GUI_WHITE);  // White color for the box
            rect.x0 = 260; rect.y0 = 120;  // Top left corner of the box
            rect.x1 = 540; rect.y1 = 320;  // Bottom right corner of the box
            GUI_AA_FillRoundedRectEx(&rect, 10);  // Fill rounded rectangle with white color

            // Set the text mode to transparent to avoid cyan behind the text
            GUI_SetTextMode(GUI_TM_TRANS);

            // Set black color for the text
            GUI_SetColor(GUI_BLACK);
            GUI_SetFont(&GUI_Font32B_ASCII);  // Use a larger font for better visibility
            GUI_DispStringAt("Chess Bot", 330, 160);  // Center the text inside the white box
            break;
        }

        case WM_NOTIFY_PARENT:
            switch (WM_GetId(pMsg->hWinSrc)) {
                case GUI_ID_BUTTON0:
                    switch (pMsg->Data.v) {
                        case WM_NOTIFICATION_CLICKED:
                            BUTTON_SetPressed(hStartButton, 1);  // Visually indicate button is pressed
                            msg.data = "newgame";
                            newgame_pub.publish(&msg);
                            nh.spinOnce();  // Ensure the message gets sent
                            WM_DeleteWindow(pMsg->hWin);  // Delete current start screen window
                            hLoadingScreen = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbLoadingScreen, 0);
                            break;
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
    }
}

static void _cbLoadingScreen(WM_MESSAGE * pMsg) {
    static BUTTON_Handle hConfirmButton;  // Confirm button to go to the next screen
    int i, j;

    switch (pMsg->MsgId) {
        case WM_CREATE:
            inLoadingScreen = true;  // Indicate that we are in the loading screen

            hConfirmButton = BUTTON_CreateEx(300, 380, 200, 50, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON0);
            BUTTON_SetText(hConfirmButton, "Confirm");
            break;

        case WM_PAINT: {
            GUI_SetBkColor(0x03989e);  // Cyan background
            GUI_Clear();

            int startX = 280, startY = 100;
            int squareSize = 30;

            for (i = 0; i < 8; i++) {
                for (j = 0; j < 8; j++) {
                    if ((i + j) % 2 == 0) {
                        GUI_SetColor(0x03989e);  // Cyan square
                    } else {
                        GUI_SetColor(GUI_BLACK);  // Black square
                    }
                    GUI_FillRect(startX + j * squareSize, startY + i * squareSize, 
                                 startX + (j + 1) * squareSize, startY + (i + 1) * squareSize);
                }
            }

            GUI_SetFont(&GUI_Font32B_ASCII);  // Larger font for pieces
            GUI_SetTextMode(GUI_TM_TRANS);  // Transparent text mode

            GUI_SetColor(GUI_WHITE);
            for (i = 0; i < 8; i++) {
                for (j = 0; j < 8; j++) {
                    char pieceStr[2] = {pieceMap[i][j], '\0'};
                    GUI_DispStringAt(pieceStr, startX + j * squareSize + 5, startY + i * squareSize);
                }
            }

            GUI_SetFont(&GUI_Font32B_ASCII);
            GUI_SetColor(GUI_WHITE);
            GUI_DispStringAt("Checking piece positions...", 240, 20);
            break;
        }

        case WM_NOTIFY_PARENT:
            switch (WM_GetId(pMsg->hWinSrc)) {
                case GUI_ID_BUTTON0:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        WM_DeleteWindow(pMsg->hWin);  // Delete current window
                        hDifficultyScreen = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbDifficultyScreen, 0);
                        inLoadingScreen = false;  // We have left the loading screen
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
    }
}

/* Window handler for the difficulty selection screen */
static void _cbDifficultyScreen(WM_MESSAGE * pMsg) {
    GUI_RECT rect;
    int i;

    switch (pMsg->MsgId) {
        case WM_CREATE: {
            hbotdifficultytext = TEXT_CreateEx(200, 50, 400, 40, pMsg->hWin, WM_CF_SHOW, TEXT_CF_HCENTER | TEXT_CF_VCENTER, GUI_ID_TEXT0, "Chess Bot Difficulty");
            TEXT_SetFont(hbotdifficultytext, &GUI_Font32B_ASCII);
            TEXT_SetTextColor(hbotdifficultytext, GUI_WHITE);
            hSlider = SLIDER_CreateEx(200, 140, 400, 60, pMsg->hWin, WM_CF_SHOW, SLIDER_CF_HORIZONTAL, GUI_ID_SLIDER0);  // Larger slider
            SLIDER_SetRange(hSlider, 0, 20);
            SLIDER_SetValue(hSlider, sliderValue);
            SLIDER_SetNumTicks(hSlider, 21);

            for (i = 0; i <= 20; i++) {
                char valueStr[4];
                sprintf(valueStr, "%d", i);
                GUI_DispStringAt(valueStr, 200 + (i * 18), 90);
            }

            hWhiteButton = BUTTON_CreateEx(125, 230, 150, 60, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON1);
            BUTTON_SetText(hWhiteButton, "White");
            BUTTON_SetBkColor(hWhiteButton, BUTTON_CI_UNPRESSED, GUI_WHITE);
            BUTTON_SetBkColor(hWhiteButton, BUTTON_CI_PRESSED, GUI_LIGHTGRAY);
            BUTTON_SetTextColor(hWhiteButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hWhiteButton, &GUI_Font32B_ASCII); 
            hBlackButton = BUTTON_CreateEx(325, 230, 150, 60, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON2);
            BUTTON_SetText(hBlackButton, "Black");
            BUTTON_SetBkColor(hBlackButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetBkColor(hBlackButton, BUTTON_CI_PRESSED, GUI_GRAY);
            BUTTON_SetTextColor(hBlackButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hBlackButton, &GUI_Font32B_ASCII); 
            hRandomButton = BUTTON_CreateEx(525, 230, 150, 60, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON3);
            BUTTON_SetText(hRandomButton, "Random");
            BUTTON_SetBkColor(hRandomButton, BUTTON_CI_UNPRESSED, GUI_LIGHTGRAY);
            BUTTON_SetBkColor(hRandomButton, BUTTON_CI_PRESSED, GUI_GRAY);
            BUTTON_SetTextColor(hRandomButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hRandomButton, &GUI_Font32B_ASCII); 
            hConfirmButton = BUTTON_CreateEx(250, 350, 300, 60, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON4);
            BUTTON_SetText(hConfirmButton, "Confirm");
            BUTTON_SetBkColor(hConfirmButton, BUTTON_CI_UNPRESSED, GUI_WHITE);
            BUTTON_SetBkColor(hConfirmButton, BUTTON_CI_PRESSED, GUI_LIGHTGRAY);
            BUTTON_SetTextColor(hConfirmButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hConfirmButton, &GUI_Font32B_ASCII);
            GUI_SetColor(GUI_WHITE);
            GUI_DrawRoundedRect(150, 80, 650, 370, 10);  // White box around slider and buttons
            break;
        }

        case WM_PAINT:
            GUI_SetBkColor(0x03989e);
            GUI_Clear();
            GUI_SetTextMode(GUI_TM_TRANS);
            
            break;

        case WM_NOTIFY_PARENT:
            switch (WM_GetId(pMsg->hWinSrc)) {
                case GUI_ID_SLIDER0:
                    if (pMsg->Data.v == WM_NOTIFICATION_VALUE_CHANGED) {
                        sliderValue = SLIDER_GetValue(hSlider);
                    }
                    break;

                case GUI_ID_BUTTON1:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        strcpy(selectedColor, "White");
                    }
                    break;

                case GUI_ID_BUTTON2:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        strcpy(selectedColor, "Black");
                    }
                    break;

                case GUI_ID_BUTTON3:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        strcpy(selectedColor, "Random");
                    }
                    break;

                case GUI_ID_BUTTON4:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        msg.data = selectedColor;
                        playercolor_pub.publish(&msg);
                        msg.data = std::to_string(sliderValue).c_str();
                        difficulty_pub.publish(&msg);
                        msg.data = "start";
                        startgame_pub.publish(&msg);
                        nh.spinOnce();
                        WM_DeleteWindow(pMsg->hWin);
                        hPlayingScreen = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbPlayingScreen, 0);
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
    }
}

/* Window handler for the playing screen */
static void _cbPlayingScreen(WM_MESSAGE * pMsg) {
    int i, j;

    switch (pMsg->MsgId) {
        case WM_CREATE: {
            hStatusText = TEXT_CreateEx(260, 30, 300, 40, pMsg->hWin, WM_CF_SHOW, TEXT_CF_HCENTER | TEXT_CF_VCENTER, GUI_ID_TEXT0, "White's Turn");
            TEXT_SetFont(hStatusText, &GUI_Font32B_ASCII);
            TEXT_SetTextColor(hStatusText, GUI_WHITE);
            hEndTurnButton = BUTTON_CreateEx(500, 150, 200, 80, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON1);
            BUTTON_SetText(hEndTurnButton, "End Turn");
            BUTTON_SetBkColor(hEndTurnButton, BUTTON_CI_UNPRESSED, GUI_WHITE);
            BUTTON_SetBkColor(hEndTurnButton, BUTTON_CI_PRESSED, GUI_LIGHTGRAY);
            BUTTON_SetTextColor(hEndTurnButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hEndTurnButton, &GUI_Font32B_ASCII);  // Set a larger font size for button text

            hForfeitButton = BUTTON_CreateEx(500, 250, 200, 80, pMsg->hWin, WM_CF_SHOW, 0, GUI_ID_BUTTON2);
            BUTTON_SetText(hForfeitButton, "Forfeit");
            BUTTON_SetBkColor(hForfeitButton, BUTTON_CI_UNPRESSED, GUI_WHITE);
            BUTTON_SetBkColor(hForfeitButton, BUTTON_CI_PRESSED, GUI_LIGHTGRAY);
            BUTTON_SetTextColor(hForfeitButton, BUTTON_CI_UNPRESSED, GUI_BLACK);
            BUTTON_SetFont(hForfeitButton, &GUI_Font32B_ASCII);  // Set a larger font size for button text
            break;
        }

        case WM_PAINT: {
            GUI_SetBkColor(0x03989e);
            GUI_Clear();

            int startX = 50, startY = 80;
            int squareSize = 40;

            for (i = 0; i < 8; i++) {
                for (j = 0; j < 8; j++) {
                    if ((i + j) % 2 == 0) {
                        GUI_SetColor(0x03989e);  // Cyan square
                    } else {
                        GUI_SetColor(GUI_WHITE);  // Black square
                    }
                    GUI_FillRect(startX + j * squareSize, startY + i * squareSize, startX + (j + 1) * squareSize, startY + (i + 1) * squareSize);
                }
            }

            GUI_SetFont(&GUI_Font32B_ASCII);
            GUI_SetTextMode(GUI_TM_TRANS);
            GUI_SetColor(GUI_BLACK);
            for (i = 0; i < 8; i++) {
                for (j = 0; j < 8; j++) {
                    char pieceStr[2] = {pieceMap[i][j], '\0'};
                    GUI_DispStringAt(pieceStr, startX + j * squareSize + 10, startY + i * squareSize +5);
                }
            }
            break;
        }

        case WM_NOTIFY_PARENT:
            switch (WM_GetId(pMsg->hWinSrc)) {
                case GUI_ID_BUTTON1:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        msg.data = "end";
                        end_turn_pub.publish(&msg);
                        nh.spinOnce();
                    }
                    break;

                case GUI_ID_BUTTON2:
                    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
                        msg.data = "Forfeit? Nah try harder next time.";
                        endgame_pub.publish(&msg);
                        nh.spinOnce();
                        WM_DeleteWindow(pMsg->hWin);
                        //WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbStartScreen, 0);
                    }
                    break;//hWin
            }
            break;

        default:
            WM_DefaultProc(pMsg);
    }
}

void setup() {
    GUI_Init();

    nh.initNode();
    nh.advertise(newgame_pub);
    nh.advertise(end_turn_pub);
    nh.advertise(endgame_pub);
    nh.advertise(playercolor_pub);
    nh.advertise(difficulty_pub);
    nh.advertise(startgame_pub);

    nh.subscribe(pieceboard_sub);
    nh.subscribe(gamestatus_sub);
    nh.subscribe(endgame_sub);  // Subscribe to the "endgame" topic

    LCD_ROTATE_SetSel(1);
    WM_MULTIBUF_Enable(1);

    WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN, WM_CF_SHOW, _cbStartScreen, 0);
}

void loop() {
    nh.spinOnce();
    GUI_Exec();
    GUI_Delay(50);
}
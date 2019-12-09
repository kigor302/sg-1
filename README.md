# Example of Playing and Recording music.

This project performs a combination of the following actions at the same time:

- play wav song from SD card
- record a wav file to SD card 

## Compatibility

| ESP32-LyraT | ESP32-LyraTD-MSC | ESP32-LyraT-Mini | ESP32-AI-Thinker
|:-----------:|:----------------:|:----------------:| :---------------:
| [![alt text](../../../docs/_static/esp32-lyrat-v4.3-side-small.jpg "ESP32-LyraT")](https://docs.espressif.com/projects/esp-adf/en/latest/get-started/get-started-esp32-lyrat.html) | [![alt text](../../../docs/_static/esp32-lyratd-msc-v2.2-small.jpg "ESP32-LyraTD-MSC")](https://docs.espressif.com/projects/esp-adf/en/latest/get-started/get-started-esp32-lyratd-msc.html) |[![alt text](../../../docs/_static/esp32-lyrat-mini-v1.2-small.jpg "ESP32-LyraT-Mini")](https://docs.espressif.com/projects/esp-adf/en/latest/get-started/get-started-esp32-lyrat-mini.html) |
| ![alt text](../../../docs/_static/yes-button.png "Compatible") | ![alt text](../../../docs/_static/yes-button.png "Compatible") |![alt text](../../../docs/_static/yes-button.png "Compatible") |

## Usage

Prepare the audio board:

- Connect speakers or headphones to the board.
- Insert a microSD card into board's slot.

Configure the example:

- Select compatible audio board in `menuconfig` > `Audio HAL`.

Load and run the example:

- The board will start and show the option to select 3 modes Volume, Play and Record track selection.
- Press the mode key to chose the option to select.
- Press Play button to play selected track.
- Press Record to record selected track from microphones.
- Both Play and Record possible but not on the same track.
- Use + and - Button to chose right selection.
- Press Set button to stop all playing and recording.
- When 2 times play button pressed the music is holded or resumed
 

## Note

You will be able to configure input codec channel (as MIC or LINE-IN) in menuconfig.

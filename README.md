# MusicVisualizer
Documented for Capstone at Humber College Electronics Engineering Technology Program

To start Arduino:
  - Download the Audio_FFT file and import the zip libraires.
  - Microphone output should be connected to pin A0 (or you could change it on the program)
  - Arduino will receive audio values automatically
LED matrix should have all the libraries installed. Please visit : https://github.com/adafruit/rpi-rgb-led-matrix
Then add the bargraph file under where you have installed the libraries, preferably under rpi-rgb-led-matrix/example-api-use

Running Project:
  - To run the project there's a python script that needs to be installed on the Pi and run it every time before running the whole project.
  - Run the python script by: sudo python "nameofthescript".py
  - Then you can start running the main bargraph file (you should be in the same directory to run it) by: ./bragraph
  - Notice that sudo command does not work for the program
  - To stop the program simply try: Ctrl + C


  

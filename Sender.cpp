/* Author Name: Andersen Lin
 * Email: andersen1997801@gmail.com
 * Discription: this is a demo code for that works with Nuitrack SDK and ROS
 * it rotates a gimbal based on the inputs from joint positions and collaborates with a robot to trace a human body 
 * To Compile: g++ -std=c++11 Sender.cpp -o Sender -lserial
 * To Run: sudo ./Sender
 */

#include <serial/serial.h>

#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace std;
#define PORT 8080
#define PI 3.1415926535

void createSocket();
double readData();
double pidControl(double, double, double, double, double);

int client_fd;
double valread;

//Create char buffer to store transmitted data
char buffer[1024] = {0};

// PID parameters
double kp, ki, kd;
double currentYaw;
double err, derivative;
double lastError = 0;
double integ = 0;
double targetYaw = 0;
double pidYaw;

// Log file that communicates gimbal and robot
FILE * logFile;

// Create a server that listens to a port on localhost
void createSocket() {

	int server_fd;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);

	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family      = AF_INET;
	address.sin_addr.s_addr = inet_addr("127.0.0.1");
	address.sin_port        = htons(PORT);

	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	if (listen(server_fd, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}

	if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}
}

// Calculate degree error as an PID input
double readData() {

  bzero(buffer, 1024);
  read(client_fd, buffer, 1024);

  // Parse a comma-separated coordinate
  double degree;
  string torsoX, torsoZ;
  string stread = string(buffer);
  stringstream ssread(stread);
  bool isFirstRead = false;

  while (ssread.good()) {
    string substr;
    getline(ssread, substr, ',');

    if (isFirstRead) {
      torsoZ = substr;
    } else {
      torsoX = substr;
      isFirstRead = true;
    }
  }

  // Calculate the angle to rotate on yaw axis
  degree = asin(atof(torsoX.c_str()) / atof(torsoZ.c_str())) * 180.0 / PI;

  return degree;
}

double pidControl(double tarVal, double curVal, double kp, double ki, double kd) {
	err = tarVal - curVal;
	integ = integ + err;
	derivative = err - lastError;

	double outp = (kp * err) + (ki * integ) + (kd * derivative);
	lastError = err;

	return outp;
}

int main() {

  // Initialize serial communication
  string port = "/dev/ttyUSB0";
  unsigned long baud = 115200;
  serial::Serial gim_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";

  if (gim_serial.isOpen()) {
    cout << " Yes." << endl;
  } else {
    cout << " No." << endl;
  }

  // Initialize a set yaw command as a default value
  uint8_t send[7] = {0xFA, 0x02, 0x0C, 0xDC, 0x05, 0x33, 0x34};

  int yawAngle, yawRaw;
  stringstream yawStringStream;
  string yawString;

  // Reset the angle to zero degree
  gim_serial.write(send, 7);

  // Establish socket communication
  createSocket();

  // Tuned PID parameters
  kp = 1.0;
  ki = 0;
  kd = 0;

  // Auto body centering by collaboration of gimbal and robot
  while (1) {
    currentYaw = readData();
    pidYaw = pidControl(targetYaw, currentYaw, kp, ki, kd);
    cout << "Current yaw: " << currentYaw << ", " << "PID yaw: " << pidYaw << endl;

    if (fabs(pidYaw) <= 5) { // Within threadshold, rotate the gimbal
      yawRaw = 1500 + 2 * pidYaw; // Convert to a decimal value from 700 to 2300
      yawStringStream << hex << yawRaw; // Convert to a hexadecimal value
      yawString = yawStringStream.str(); // Convert to a string
      yawStringStream.str(""); // Reset the stringstream for next iteration

      // Divide the hex string into 2 bytes
      int yawHighByte = stoi(yawString.substr(0, 1), nullptr, 16);
      int yawLowByte = stoi(yawString.substr(1, 2), nullptr, 16);
      send[3] = yawLowByte;
      send[4] = yawHighByte;;

			gim_serial.write(send, 7);

    } else if (fabs(pidYaw) > 5) { // Exceeds threadshold, rotate the robot
      cout << "Angle exceeds the limit (+- 5 degrees)" << endl;

			// Write to the log file
			int fd = open("logFile", O_WRONLY | O_DSYNC | O_CREAT, 0666);
			logFile = fdopen(fd, "w");
			fprintf(logFile, "%lf", currentYaw);
			fclose(logFile);

    } else { // Ignore nan values (when no torso joint is captured)
			continue;
		}

    //usleep(20000); // No need to sleep as the sending rate of Nuitrack is the same as its frame rate (30 fps)
  }

  return 0;
}

/* References:
 * STorM32-BGC - Pins and Connectors: http://www.olliw.eu/storm32bgc-v1-wiki/Pins_and_Connectors#UART
 * USB to TTL Serial Cable: https://media.digikey.com/pdf/Data%20Sheets/Adafruit%20PDFs/954_Web.pdf
 * Serial Port library written in C++: http://wjwwood.io/serial/doc/1.1.0/index.html
 * Serial Communication - RC Commands: http://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication
 */

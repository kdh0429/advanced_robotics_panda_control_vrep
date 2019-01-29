#include <iostream>
#include <string>
#include <conio.h>
#include "vrep_bridge.h"

#include "controller.h"

using namespace std;
int main()
{
	VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	//VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	const double hz = 1000;
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData(vb.getPosition(), vb.getVelocity(), vb.getTorque());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			ac.readData(vb.getPosition(), vb.getVelocity());
			cout << "Initial q: " << vb.getPosition().transpose() << endl;
			is_first = false;
			ac.initPosition();
		}

		if (_kbhit())
		{
			int key = _getch();

			switch (key)
			{
				// Implement with user input
			case 'i':
				ac.setMode("joint_ctrl_init");
				break;
			case 'h':
				ac.setMode("joint_ctrl_home");
				break;
			case 'z':
				ac.setMode("hw2_1");
				break;
			case 'x':
				ac.setMode("hw2_2");
				break;
			case 'c':
				ac.setMode("hw2_3");
				break;
			case 'v':
				ac.setMode("hw3_1");
				break;
			case 'b':
				ac.setMode("hw3_2");
				break;
			case 'n':
				ac.setMode("hw3_3");
				break;
			case 'a':
				ac.setMode("hw4_1");
				break;
			case 's':
				ac.setMode("hw4_2");
				break;
			case 'd':
				ac.setMode("hw4_3");
				break;
			case 'f':
				ac.setMode("hw5_1");
				break;
			case 'g':
				ac.setMode("hw5_2");
				break;
			case 'j':
				ac.setMode("hw6_1");

			case 'p':
				ac.setMode("hw8_1");
				break;
			case 'o':
				ac.setMode("hw8_2");
				break;
			case 'l':
				ac.setMode("hw8_3");
				break;
			case 'k':
				ac.setMode("hw9_1");
				break;

			case 't':
				ac.setMode("torque_ctrl_dynamic");
				break;



			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				ac.closeFile();
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			vb.setDesiredPosition(ac.getDesiredPosition());
			vb.setDesiredTorque(ac.getDesiredTorque());
		
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}

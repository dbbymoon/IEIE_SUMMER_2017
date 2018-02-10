/*
거리단위 m, 시간단위 s
*/
#include <iostream>	// input & output stream
#include <string>		// string for file name
#include <iomanip>	// input & output maipulation
#include <fstream>		// file stream
#include<cmath>		// mathematic function

using namespace std;

struct Point {
	double x, y;
};

extern double unirv(void);
extern double Normal(void);
extern int Bernoulli(double seed, double r);
extern double u_dist(const Point& p1, const Point& p2);
extern Point brown(const Point& p1, double distance);

// cout error modified (operator <<)
#ifdef WORKAROUND
std::ostream& operator<<(std::ostream& os, __int64 i)
{
	char buf[20];
	sprintf(buf, "%I64d", i);
	os << buf;
	return os;
}
#endif

/* environment */
#define M 10													// # of sensor node in the coverage
#define DAY 30												// # of experience time
#define MINI_SLOT 1										// = 400 micro second
#define SLOT (2 * MINI_SLOT)						// packet transmition time = 4 milli second
#define SIMULATION_TIME 1800000			// is 1 hour (1trip = 300,000 slots = 600,000 mini slots)

typedef __int64 Clocktype; //simulation clock
typedef enum { Hibernation = 0, Detecting = 1, Carrier_sense = 2, Backing_off = 3, Transmitting = 4 }NodeMode; //node's mode
typedef enum { wait = 0, transmit = 1, fail = 2 }TransmittingState; // channal state

																	/* simulation parameters */
__int64 timer_sensor[M] = { 0 };		// timer for sensor(at the limit time, then change mode)
NodeMode mode_sensor[M] = { Hibernation };			// mode of sensor
TransmittingState transmitting_state[M] = { wait };		// Transmitting state

														/* simulation constant */
const int beacon_time = 10;										// sink sends beacon signal
const double PI = std::acos(-1);									// = 3.141592
const double V = 60e3;												// Sink speed = 60 km/h
double V_sensor = 20e3;												// sensor speed = 3, 6, 12, 20 km/h
const double Trip_distance = 20e3;							// Sink trip distance = 20 km
const double r = Trip_distance / (2 * PI);					// Radius of Trip
const double R = 2 * r;												// Radius of field
const double packet_transmission_time = 4e-3;		// = 1 slot
const double Sink_tick_angle = 360 / (Trip_distance / V * 60 * 60 / packet_transmission_time * SLOT);
const double Hight = 50;											// Sink hight
const double Threshold = r;										// Sink threshold for receive packet
double sigma = V_sensor / 3600 * 4e-3 / SLOT;

__int64 hibernation_time = r / (V / 3600) / packet_transmission_time * SLOT;								// limit time of hibernation mode
const int detecting_time = 10 * SLOT;								// limit time of detecting mode to listen beacon signal
const int backing_off_time = 10 * SLOT;							// limit time of backing-off mode
const int transmitting_time = SLOT;									// limit time of transmitting mode


																	/* simulation variables */
__int64 tick;		// simulation clock
int Node = M;		// #_sensor node for simulation
int n = 0;				// current node
int beacon = 0;	//
double p = 0.5;	// transmission attempt probability
double angle = 0;
__int64 Fail = 0;
__int64 Fail_1 = 0;	// test
__int64 Fail_2 = 0;	// test
__int64 Arrived = 0;
__int64 Received = 0;
long double Throughput = 0;

/* power */
double power[M] = { 0 };
const double frequency = 2400;
const double SIR_threshold = 4;
const double height_base = 50;
const double height_mobile = 0;
double SIR = 0;
double interference = 0;
__int64 h_power = 0;
__int64 d_power = 0;
__int64 c_power = 0;
__int64 b_power = 0;
__int64 t_power = 0;

Point location_sensor[M];
Point sink;

void main() {
	string filename;
	filename = "C:\\Users\\NARAE MOON\\Desktop\\";
	filename += "IEIE_SUMMER_2017";
	filename += "_Th_vs_hibernationTime";
	filename += "_M=";
	filename += std::to_string(Node);
	filename += "_V_sensor=";
	filename += std::to_string(V_sensor);
	filename += "_";
	filename += std::to_string(DAY);
	ofstream outFile(filename + ".txt");

	outFile << "Defaults:" << " M = " << Node << " DAY = " << DAY << " SIMULATION_TIME(0.4msec) = " << SIMULATION_TIME << "slot = " << SLOT << endl;
	outFile << "hibernation_time \t Node \t Arrived \t Received \t Fail \t Fail_1 \t Fail_2 \t Throughput_M=" << Node << "(Packets/Slot)" << " \t  h_power" << " \t d_power" << " \t c_power" << " \t b_power" << " \t t_power" << endl;

	for (hibernation_time = pow(2, 0); hibernation_time <= pow(2, 10); hibernation_time *= 2) {
		//for (V_sensor = 6e3; V_sensor <= 6e3; V_sensor += 6e3) {
		// initial environment
		sigma = V_sensor / 3600 * 4e-3 / SLOT;

		Arrived = 0;
		Received = 0;
		h_power = 0;
		d_power = 0;
		b_power = 0;
		t_power = 0;
		Fail = 0;
		Fail_1 = 0;
		Fail_2 = 0;

		for (int day = 0; day < DAY; day++) {
			angle = 0;
			beacon = 0;

			for (n = 0; n < Node; n++) {
				//location_sensor[n] = { 0,0 };
				do {
					location_sensor[n].x = Normal() * R;
					location_sensor[n].y = Normal() * R;
				} while (u_dist({ 0,0 }, location_sensor[n]) > R);
			}
			for (n = 0; n < Node; n++) {
				mode_sensor[n] = Hibernation;
			}
			for (n = 0; n < Node; n++) {
				timer_sensor[n] = 0;
			}
			for (n = 0; n < Node; n++) {
				transmitting_state[n] = wait;
			}

			for (tick = 0; tick < SIMULATION_TIME; tick++) {		// increase simulation time
																	// Positioning of sink node
				sink = { r , angle };
				angle += Sink_tick_angle;

				// pilot signal
				if (tick % beacon_time == 0) {
					beacon = 1;
				}
				else {
					beacon = 0;
				}

				// Positioning of sensor nodes
				for (n = 0; n < Node; n++) {
					do {
						Point temp = { 0, 0 };
						// distance in brown() = velocity of sensor node * mini-slot time length(= 0.4msec)
						temp = brown(location_sensor[n], sigma);
						if (u_dist({ 0,0 }, temp) > R) {
							cout << temp.x << ", " << temp.y << " : " << location_sensor[n].x << ", " << location_sensor[n].y << endl;
							cout << "Error: Positioning of sensor node_" << n << endl;
						}
						else {
							location_sensor[n] = temp;
						}
					} while (u_dist({ 0,0 }, location_sensor[n]) > R);
				}


				for (n = 0; n < Node; n++) {
					// [Hibernation mode]
					if (mode_sensor[n] == Hibernation) {
						if (timer_sensor[n] >= hibernation_time) { // <Hibernation time ends>
							mode_sensor[n] = Detecting;
							timer_sensor[n] = 0;
						}
					}

					// [Detecting mode]
					if (mode_sensor[n] == Detecting) {
						if (timer_sensor[n] < detecting_time) {	// <to detect beacon signal>
							if (beacon == 1) {
								if (u_dist(sink, location_sensor[n]) <= Threshold) {
									mode_sensor[n] = Carrier_sense;
									timer_sensor[n] = 0;
								}
							}
						}
						else { // <Detecting time ends>
							mode_sensor[n] = Hibernation;
							timer_sensor[n] = 0;
						}
					}

					// [Carrier_sense mode]
					else if (mode_sensor[n] == Carrier_sense) {
						if (timer_sensor[n] < backing_off_time) {
							int carrier = 0;
							for (int i = 0; i < Node; i++) {
								if (u_dist(location_sensor[n], location_sensor[i]) <= Threshold) {
									if (mode_sensor[i] == Transmitting) {
										carrier += 1;
									}
								}
							}

							if (carrier == 0) {
								Arrived += 1;
								mode_sensor[n] = Transmitting;
								timer_sensor[n] = 0;
								transmitting_state[n] = transmit;
							}
							else {
								mode_sensor[n] = Backing_off;
							}
						}
						else {// <Backing_off time ends>
							mode_sensor[n] = Hibernation;
							timer_sensor[n] = 0;
						}
					}

					// [Backing_off mode]
					else if (mode_sensor[n] == Backing_off) {
						if (timer_sensor[n] < backing_off_time) {
							if (Bernoulli(unirv(), p) == 1) {
								mode_sensor[n] = Carrier_sense;
							}
						}
						else { // <Backing_off time ends>
							mode_sensor[n] = Hibernation;
							timer_sensor[n] = 0;
						}
					}
				}

				// Caculates received power at Sink node
				for (n = 0; n < Node; n++) {
					power[n] = 69.55 + 26.16 * log10(frequency) - 13.82 * log10(height_base) - 0.8 - (1.1 * log10(frequency) - 0.7) * height_mobile + 1.56 * log10(frequency) + (44.9 - 6.55 * log10(height_base)) * log10(u_dist(sink, location_sensor[n])) - 4.78 * pow(log10(frequency), 2) + 18.33 * log10(frequency) - 40.94;	 // Hata model for open areas
				}

				// Caculate Interference of SIR
				interference = 0;
				for (n = 0; n < Node; n++) {
					if (transmitting_state[n] != wait) {
						interference += power[n];
					}
				}

				//  count number of transmitting sensor
				int count = 0;
				for (int n = 0; n < Node; n++) {
					if (transmitting_state[n] != wait) {
						if (u_dist(sink, location_sensor[n]) <= Threshold) {
							count += 1;		// 전송중인 센서 개수 확인
						}
					}
				}

				for (n = 0; n < Node; n++) {
					// [Transmitting mode]
					if (mode_sensor[n] == Transmitting) {
						if (timer_sensor[n] < transmitting_time) {
							// check fail
							if (transmitting_state[n] == transmit) {
								if (count > 1) {
									SIR = power[n] / (interference - power[n]);
									if (SIR < SIR_threshold) {
										transmitting_state[n] = fail;	// SIR fail
										Fail_1 += 1;
									}
								}

								if (u_dist(sink, location_sensor[n]) > Threshold) {
									transmitting_state[n] = fail;	// Distence fail
									Fail_2 += 1;
								}
							}
						}
						else {
							if (transmitting_state[n] == fail) {
								Fail += 1;
							}
							else {
								Received += 1;
							}
							mode_sensor[n] = Hibernation;
							timer_sensor[n] = 0;
							transmitting_state[n] = wait;
						}
					}
				}

				// time goes ...
				for (n = 0; n < Node; n++) {
					timer_sensor[n] += 1;
					switch (mode_sensor[n]) {
					case 0:
						h_power += 1;
						break;
					case 1:
						d_power += 1;
						break;
					case 2:
						c_power += 1;
						break;
					case 3:
						b_power += 1;
						break;
					case 4:
						t_power += 1;
						break;
					}
				}
				if (tick % (SIMULATION_TIME / 100) == 0) {
					std::cout << "hibernation_time: " << hibernation_time << " DAY: " << day + 1 << "/" << DAY << " Simulation: " << tick / (long double)SIMULATION_TIME * 100.0 + 1 << "% Completed \r";
				}
			}
		}
		// Simulation result
		Throughput = Received / (long double)SIMULATION_TIME / (long double)DAY;
		for (n = 0; n < Node; n++) {	// test code
			cout << n << ": " << n << "(" << location_sensor[n].x << ", " << location_sensor[n].y << ")\t";
		}
		cout << endl;
		cout << Throughput << endl;
		outFile << hibernation_time << "\t" << Node << "\t" << Arrived << "\t" << Received << "\t" << Fail << "\t" << Fail_1 << "\t" << Fail_2 << "\t" << Throughput << "\t" << h_power << "\t" << d_power << "\t" << c_power << "\t" << b_power << "\t" << t_power << endl;
	}

	outFile.close();
}
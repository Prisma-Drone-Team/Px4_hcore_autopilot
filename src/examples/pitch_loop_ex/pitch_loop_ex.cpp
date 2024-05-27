/**
 * @file pitch_loop_ex.cpp
 * Debug application for pitch setpoint and open/close loop control.
 *
 * @author Michele Marolla <mic.marolla@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/vehicle_attitude.h>

#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>


extern "C" __EXPORT int pitch_loop_ex_main(int argc, char *argv[]);

matrix::Vector<float, 4> pitch_coefficients(float start, float end, float tf){
	matrix::Vector<float,4> coeff;
	coeff(0) = start;
	coeff(1) = 0.0f;
	coeff(3) = (start - end) * 2 / powf(tf,3);
	coeff(2) = -1.5f * coeff(3) * tf;

	return coeff;
}

void print_usage(){
	PX4_INFO("Usage: pitch_loop_ex -p <pitch_deg> <time_seconds>: publish pitch setpoint \
		\npitch_loop_ex -o <time_seconds>: open pitch loop for specified seconds");
}


int pitch_loop_ex_main(int argc, char *argv[]){
	if(argc < 3 || argc > 4){
		print_usage();
		return 0;
	}

	// Advertise named debug value
	struct debug_key_value_s debug_key;
	strncpy(debug_key.key, "", 10);
	debug_key.value = 0.0f;
	uORB::Publication<debug_key_value_s> pub_debug_key {ORB_ID(debug_key_value)};

	uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	bool publish_sp = false;
	bool publish_open = false;
	float des_pitch = 0.0f;
	float pitch = 0.0f;
	float dt = 0.01f;
	float tf = 0.0f;
	hrt_abstime stime;

	// Setpoint
	if(strcmp(argv[1], "-p") == 0){
		if(argc != 4){
			print_usage();
			return 0;
		}
		des_pitch = (float)atof(argv[2]) * M_DEG_TO_RAD_F;
		tf = (float)atof(argv[3]);

		// Wait for pitch measure
		bool ok = false;
		stime = hrt_absolute_time();
		while(hrt_absolute_time() - stime < 5000){
			if(vehicle_attitude_sub.updated()){
				vehicle_attitude_s att;
				if(vehicle_attitude_sub.copy(&att)){
					ok = true;
					pitch = matrix::Eulerf(matrix::Quatf(att.q)).theta();
					break;
				}
			}
		}
		if(!ok){
			PX4_ERR("Cannot read vehicle_attitude. This is REALLY bad. Aborting.");
			return 1;
		}

		publish_sp = true;
		PX4_INFO("Publishing %2.2f pitch in %2.2f, starting from %2.2f", (double)des_pitch, (double)tf, (double)pitch);
	}

	// Open loop
	else if(strcmp(argv[1], "-o") == 0){
		if(argc != 3){
			print_usage();
			return 0;
		}

		tf = (float)atof(argv[2]);
		publish_open = true;
		PX4_INFO("Opening pitch loop for %2.2f seconds", (double)tf);
	}

	else{
		print_usage();
		return 0;
	}

	// Check on tf
	if(tf <= 0){
		PX4_ERR("Timespan must be greater than zero. Aborting.");
		return 1;
	}

	// At this point, one of publish_pitch or publish_open must be true

	// Compute steps and pitch coefficients
	int steps = lround(tf/dt);
	matrix::Vector4f coeff;

	if(publish_open)
		strncpy(debug_key.key, "open_loop", 10);
	else if(publish_sp)
		coeff = pitch_coefficients(pitch, des_pitch, tf);

	stime = hrt_absolute_time();

	int count = 0;
	while(count < steps){
		// limit rate to 100hz (10ms)
		if(hrt_absolute_time() - stime > 10000){

			// update setpoint
			if(publish_sp)
				debug_key.value = pitch + (des_pitch - pitch) * count / steps;	// update setpoint

			debug_key.timestamp = hrt_absolute_time();
			pub_debug_key.publish(debug_key);

			count++;
			stime = hrt_absolute_time();
		}
		else
			px4_usleep(1000);
	}

	PX4_INFO("Done");


	return 0;
}

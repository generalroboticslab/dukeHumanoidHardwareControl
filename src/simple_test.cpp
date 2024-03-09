/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include <pthread.h>
#include <thread> // TODO

#include <iostream>
#include <sstream>

#include "ethercat.h"
#include "ethercat_motor.h"

std::vector<double> parse_doubles(const char *input)
{
   std::stringstream ss(input);
   std::vector<double> result;
   result.reserve(NUM_TARGET); // Pre-allocate space if NUM_TARGET is known

   std::string substr;
   while (std::getline(ss, substr, ','))
   {
      std::istringstream iss(substr);
      double value;
      if (!(iss >> value))
      {
         throw std::runtime_error("Invalid number format in input string");
      }
      result.push_back(value);
   }

   return result;
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
   // get arguments from command line, contains the adapter,type of control mode: csp, csv,target position or target velocity,
   // if is csp mode, the second argument is the target position, if is csv mode, the second argument is the target velocity
   // if there is no argument, the program will print the available control mode and exit

   int8_t control_mode;

   if (argc < 6)
   {
      // print argc and all argv
      printf("argc: %d\n", argc);
      for (int i = 0; i < argc; i++)
      {
         printf("argv[%d]: %s\n", i, argv[i]);
      }
      ec_adaptert *adapter = NULL;
      printf("Usage: simple_test ifname1 control_mode target max_velocity[rad/s] max_torque[0.1%%]\n");
      printf("\nAvailable adapters:\n");
      adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
      printf("control_mode: csp, csv, cst\n");
      printf("target: target position [rad]   if control_mode == csp\n");
      printf("        target velocity [rad/s] if control_mode == csv\n");
      printf("        target torque [0.1%% of rated current] if control mode == cst\n");
      return (0);
   }
   // else if the mode is csp,check the string, wheather is "csp", the target position is the third argument
   // else if the mode is csv,check the string, wheather is "csv", the target velocity is the third argument
   else
   {
      std::string modeStr = argv[2];
      std::vector<double> target_input_in = parse_doubles(argv[3]);

      double target = atof(argv[3]);
      double max_velocity = atof(argv[4]);
      double max_torque = atof(argv[5]);

      if (modeStr == "csp")
      {
         control_mode = CONTROL_MODE::CYCLIC_SYNC_POSITION;
         printf("Control mode: csp\n");
         printf("Target position [rad]: ");
      }
      else if (modeStr == "csv")
      {
         control_mode = CONTROL_MODE::CYCLIC_SYNC_VELOCITY;
         printf("Control mode: csv\n");
         printf("Target velocity [rad/s]: ");
      }
      else if (modeStr == "cst")
      {
         control_mode = CONTROL_MODE::CYCLIC_SYNC_TORQUE;
         double target_torque = atof(argv[3]);
         printf("Control mode: cst\n");
         printf("Target torque: [%%rated torue]");
      }
      else
      {
         printf("Invalid control mode\n");
         return (0);
      }

      for (double value : target_input_in)
      {
         printf("%.3f ", value);
      }
      printf("\nMax velocity: %f rad/s \n", max_velocity);
      // return (0);

      Motor motor(argv[1], control_mode, target_input_in, max_velocity,max_torque);
      motor.run();

      printf("End program\n");
      return (0);
   }
}

// sudo ./simple_test enp3s0 csv 0.1,0.2,0.4,0.8,0.1,0.2 1
//  sudo ./simple_test enp3s0 csp 0,0,0,0,0,0 1
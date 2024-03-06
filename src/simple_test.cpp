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
#include <pthread.h>
#include <math.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>


#define EC_TIMEOUTMON 500

#define NUM_TARGET 6 // num of motors

enum CONTROL_MODE : int8
{
   CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
   CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
   CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode
};

// double gear_ratio[NUM_TARGET] = {20,20,20,20,10,10};

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;
boolean SendFlag = FALSE;

// 1604 RXDO
struct TorqueOut
{
   int32 target_position; // 607A
   int32 target_velocity; // 60FF
   uint16 maximal_torque; // 6072
   uint16 control_word;   // 6040
};

// 1a03 TPDO
struct TorqueIn
{
   int32 position_actual; // 6064
   uint32 digital_inputs; // 60FD
   int32 velocity_actual; // 606C
   uint16 status_word;    // 6041
};

struct out_ELMOt              // pdo
{                             // 1c12
   int32_t target_position;   // 0x607a
   int32_t target_velocity;   // 0x60ff
   int16_t max_torque;        // 0x6072
   uint16_t control_word;     // 0x6040 pdo domain 0x1604
   int16_t target_torque;     // 0x6071
   uint8_t mode_of_operation; // 0x6060
};

struct in_ELMOt                    // pdo
{                                  // 1c13
   int32_t position_actual;        // 0x6064
   int32_t position_follow_err;    // 0x60f4
   int16_t torque_actual;          // 0x6077
   uint16_t status_word;           // 0x6041
   uint8_t mode_of_operation_disp; // 0x6061
   int32_t velocity_actual;        // 0x606C
};

// struct TorqueOut *target[NUM_TARGET];
// struct TorqueIn *val[NUM_TARGET];

struct out_ELMOt *target[NUM_TARGET];
struct in_ELMOt *val[NUM_TARGET];

void *ecatprint()
{
   //  int slave;
   while (1)
   {
      //   printf("Main Outputs:  Target: 0x%x, control: 0x%x\n", target->target_position, target->control_word);
      //   printf("Main Inputs:  Target: 0x%x, control: 0x%x\n", val->position_actual, val->status_word);
      // printf("Flag: %d\n", SendFlag);
      SendFlag = !SendFlag;
      // printf("Flag: %d\n", SendFlag);
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      usleep(1000);
   }
}

#define READ(slaveId, idx, sub, buf, comment)                                                                                                                      \
   {                                                                                                                                                               \
      buf = 0;                                                                                                                                                     \
      int __s = sizeof(buf);                                                                                                                                       \
      int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
      printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
   }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                       \
   {                                                                                                                                        \
      int __s = sizeof(buf);                                                                                                                \
      buf = value;                                                                                                                          \
      int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
      printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
   }

// CA	= FALSE = single subindex. TRUE = Complete Access, all subindexes written
#define WRITECA(slaveId, idx, sub, buf, value, comment)                                                                                     \
   {                                                                                                                                        \
      int __s = sizeof(buf);                                                                                                                \
      buf = value;                                                                                                                          \
      int __ret = ec_SDOwrite(slaveId, idx, sub, TRUE, __s, &buf, EC_TIMEOUTRXM);                                                           \
      printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
   }

#define CHECKERROR(slaveId)                                                                                                                                                                     \
   {                                                                                                                                                                                            \
      ec_readstate();                                                                                                                                                                           \
      printf("EC> \"%s\" %x - %x [%s] \n", (char *)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
   }

int ELMOsetupGOLD(ecx_contextt *context, uint16 slave)
{
   int wkc = 0;
   uint32_t sdoObj = 0x00000000;
   uint8_t disable_bits = 0x00;
   uint8_t enable_bits = 0x01;
   uint16_t objAddr = 0x0000;
   uint16_t TxAddr = 0x1607; // "RPDO8 Mapping"
   uint16_t RxAddr = 0x1A07; // "TPDO8 Mapping"

   wkc += ec_SDOwrite(slave, TxAddr, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE); // 0 disable
   sdoObj = 0x607A0020;                                                                                    // Target position, INTEGER32 (0020)
   wkc += ec_SDOwrite(slave, TxAddr, 0x01, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60FF0020; // Target velocity, INTEGER32 (0020)
   wkc += ec_SDOwrite(slave, TxAddr, 0x02, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60720010; // Max torque, UNSIGNED16 (0010)
   wkc += ec_SDOwrite(slave, TxAddr, 0x03, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60400010; // Controlword, UNSIGNED16 (0010)
   wkc += ec_SDOwrite(slave, TxAddr, 0x04, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60710010; // Target torque, INTEGER16 (0010)
   wkc += ec_SDOwrite(slave, TxAddr, 0x05, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60600008; // Modes of operation, INTEGER8 (0008)
   wkc += ec_SDOwrite(slave, TxAddr, 0x06, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   enable_bits = 0x06; //            0x00      "Number of sub indexes"  [UNSIGNED8        RWR_R_]      0x06 / 6
   wkc += ec_SDOwrite(slave, TxAddr, 0x00, FALSE, sizeof(enable_bits), &(enable_bits), EC_TIMEOUTSTATE);

   wkc += ec_SDOwrite(slave, RxAddr, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE); // 0 disable
   sdoObj = 0x60640020;                                                                                    // Position actual value
   wkc += ec_SDOwrite(slave, RxAddr, 0x01, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60F40020; // Following error actual value
   wkc += ec_SDOwrite(slave, RxAddr, 0x02, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60770010; // Torque actual value
   wkc += ec_SDOwrite(slave, RxAddr, 0x03, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60410010; // Statusword
   wkc += ec_SDOwrite(slave, RxAddr, 0x04, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x60610008; // Modes of operation display
   wkc += ec_SDOwrite(slave, RxAddr, 0x05, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   sdoObj = 0x606C0020; // Additional position actual value
   wkc += ec_SDOwrite(slave, RxAddr, 0x06, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
   enable_bits = 0x06;
   wkc += ec_SDOwrite(slave, RxAddr, 0x00, FALSE, sizeof(enable_bits), &(enable_bits), EC_TIMEOUTSTATE);

   // Tx PDO disable      // 0x1c12      "SM2(Outputs) PDO Assignment"
   wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE);
   // Tx PDO dictionary mapping
   objAddr = TxAddr;
   wkc += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(objAddr), &(objAddr), EC_TIMEOUTSTATE);

   // Rx PDO disable      // 0x1c13      "SM3(Inputs) PDO Assignment"
   wkc += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE);
   // Rx PDO dictionary mapping
   objAddr = RxAddr;
   wkc += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(objAddr), &(objAddr), EC_TIMEOUTSTATE);

   enable_bits = 0x01;
   wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(enable_bits), &(enable_bits), EC_TIMEOUTSTATE);
   wkc += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(enable_bits), &(enable_bits), EC_TIMEOUTSTATE);

   int8 op_mode = CONTROL_MODE::CYCLIC_SYNC_POSITION; // 8:position 9:velocity 10:torque
   // de0x6060      "Modes of operation"
   wkc += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(op_mode), &op_mode, EC_TIMEOUTSTATE); //    cyclic sychronous position mode
   printf("wkc : %d\n", wkc);
   printf("supported drive modes: %d\n", op_mode);

   return wkc;
}

void simpletest(char *ifname, int8_t control_mode_int8, double target_input_in[NUM_TARGET], double max_velocity)
{

   int32_t target_int32[NUM_TARGET];
   uint32_t max_velocity_uint[NUM_TARGET];
   // double gear_ratio = 20 ;
   double gear_ratio[NUM_TARGET] = {20,20,20,20,10,10};
   double target_input[NUM_TARGET];

   for (int i = 0; i < NUM_TARGET; i++)
   {
      target_input[i]=target_input_in[i]*gear_ratio[i];
      max_velocity_uint[i] = (uint32_t)(max_velocity*gear_ratio[i] / 0.10472); // 1 RPM = 0.10472 rad/s
      printf("%d %d \n ",i, max_velocity_uint[i]);
   }
   // return;


   
   if ((control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION)|| control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY)
   {
      // the range of encoder is 0~2^17-1,unit is cnt (count)
      // the target position's unit is rad
      // convert the target position from rad to cnt
      for (int i = 0; i < NUM_TARGET; i++)
      {
         target_int32[i] = (int32_t)(target_input[i] * 131072 / (2 * M_PI));
      }
   }

   // target = (struct TorqueOut*)(ec_slave[1].outputs);
   // val = (struct TorqueIn*)(ec_slave[1].inputs);
   int i, oloop, iloop, chk;
   needlf = FALSE;
   inOP = FALSE;
   uint32 buf32;
   uint16 buf16;
   uint8 buf8;
   int32 sbuf32;
   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */
      for (size_t i = 0; i < NUM_TARGET; i++)
      {
         ec_slave[1 + i].PO2SOconfigx = &ELMOsetupGOLD; 
      }

      if (ec_config_init(FALSE) > 0)
      {

         ec_configdc();
         usleep(100000);
         for (int i = 1; i <= ec_slavecount; i++)
         {
            if (ELMOsetupGOLD(&ecx_context,i) != 23)
            {
               printf("error \n");
               exit(1);
            };
         }

         printf("%d slaves found and configured.\n", ec_slavecount);

         if (forceByteAlignment)
         {
            ec_config_map_aligned(&IOmap);
         }
         else
         {
            ec_config_map(&IOmap);
         }

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

         // show slave info
         for (int i = 1; i <= ec_slavecount; i++)
         {
            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                   i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                   ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
         }

         // /** disable heartbeat alarm */
         // for (int i = 1; i <= ec_slavecount; i++) {
         //     READ(i, 0x10F1, 2, buf32, "Heartbeat?");
         //     WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

         //     WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
         //     WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
         // }
         printf("Slaves mapped, state to SAFE_OP.\n");
         /** set PDO mapping */
         /** opMode: 8  => csp */
         /** opMode: 9  => csv */
         for (int i = 0; i < ec_slavecount; i++)
         {
            READ(i+1, 0x1a03, 0, buf32, "rxPDO:0 1604");
            WRITE(i+1, 0x6060, 0, buf8, control_mode_int8, "OpMode");
            READ(i+1, 0x6061, 0, buf8, "OpMode display");

            READ(i+1, 0x1c12, 0, buf32, "rxPDO:0");
            READ(i+1, 0x1c13, 0, buf32, "txPDO:0");

            READ(i+1, 0x1c12, 1, buf32, "rxPDO:1");
            READ(i+1, 0x1c13, 1, buf32, "txPDO:1");
         }

         // int32 ob2;
         // int os;
         for (int i = 0; i < ec_slavecount; i++)
         {
            // os = sizeof(ob2);
            // ob2 = 0x16040001;
            // ec_SDOwrite(i+1, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
            // os = sizeof(ob2);
            // ob2 = 0x1a030001;
            // ec_SDOwrite(i+1, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

            READ(i+1, 0x1a07, 0, buf8, "txPDO:7");
            READ(i+1, 0x1c12, 0, buf32, "rxPDO:0");
            READ(i+1, 0x1c13, 0, buf32, "txPDO:0");

            READ(i+1, 0x1c12, 1, buf32, "rxPDO:1");
            READ(i+1, 0x1c13, 1, buf32, "txPDO:1");

            READ(i+1, 0x1604, 0, buf32, "rxPDO:0 1604");
            READ(i+1, 0x1a07, 0, buf32, "txPDO:0 0x1a07");
            READ(i+1, 0x1604, 1, buf32, "rxPDO:1 1604");
            READ(i+1, 0x1a07, 1, buf32, "txPDO:1 0x1a07");
            READ(i+1, 0x1604, 2, buf32, "rxPDO:2 1604");
            READ(i+1, 0x1a07, 2, buf32, "txPDO:2 0x1a07");
            READ(i+1, 0x1604, 3, buf32, "rxPDO:3 1604");
            READ(i+1, 0x1a07, 3, buf32, "txPDO:3 0x1a07");
            READ(i+1, 0x1604, 4, buf32, "rxPDO:4 1604");
            READ(i+1, 0x1a07, 4, buf32, "txPDO:4 0x1a07");
            READ(i+1, 0x6064, 0, sbuf32, "*position actual value*");
            READ(i+1, 0x6080, 0, buf32, "*Max motor speed*");
            WRITE(i+1, 0x6080, 0, buf32, max_velocity_uint[i], "*Max motor speed*");
            usleep(100);
            READ(i+1, 0x6080, 0, buf32, "*Max motor speed*");
         }
         //  return;
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

         for (size_t i = 0; i < NUM_TARGET; i++)
         {
            target[i] = (struct out_ELMOt *)(ec_slave[1 + i].outputs);
            val[i] = (struct in_ELMOt *)(ec_slave[1 + i].inputs);
         }

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0))
            oloop = 1;
         if (oloop > 8)
            oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0))
            iloop = 1;
         if (iloop > 8)
            iloop = 8;

         printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;

            /**
             * Drive state machine transistions
             *   0 -> 6 -> 7 -> 15
             */
            for (int i = 1; i <= ec_slavecount; i++)
            {

               READ(i, 0x6064, 0, sbuf32, "*position actual value*");
               READ(i, 0x6077, 0, buf16, "Torque actual value");
               READ(i, 0x6041, 0, buf16, "*status word*");
               if (buf16 == 0x218)
               {
                  WRITE(i, 0x6040, 0, buf16, 128, "*control word*");
                  usleep(100000);
                  READ(i, 0x6041, 0, buf16, "*status word*");
               }

               WRITE(i, 0x6040, 0, buf16, 0, "*control word*");
               usleep(100000);
               READ(i, 0x6041, 0, buf16, "*status word*");

               WRITE(i, 0x6040, 0, buf16, 6, "*control word*");
               usleep(100000);
               READ(i, 0x6041, 0, buf16, "*status word*");

               WRITE(i, 0x6040, 0, buf16, 7, "*control word*");
               usleep(100000);
               READ(i, 0x6041, 0, buf16, "*status word*");

               WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); // Fault reset
               usleep(100000);
               READ(i, 0x6041, 0, buf16, "*status word*");
               // WRITE(i, 0x6060, 0, buf8, 6, "OpMode"); usleep(100000);

               CHECKERROR(i);
               READ(i, 0x6061, 0, buf8, "OpMode");

               READ(i, 0x1001, 0, buf8, "Error");
            }

            // int reachedInitial = 0; //TODO CHAGE TO ARRAY FOR EACH MOTOR, THIS IS NOT USED

            // osal_thread_create(&thread2, 12800, &ecatprint,NULL);
            /* cyclic loop for two slaves*/

            for (size_t i = 0; i < NUM_TARGET; i++)
            {
               // SET INITAL GOAL
               target[i]->control_word = 0; // stop
               target[i]->max_torque = (uint16)(1000000);
               if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION) // csp
               {
                  // READ(1, 0x6080, 0, buf32, "*max velocity*");
                  // WRITE(1, 0x6080, 0, buf32, 100000, "*max velocity*"); usleep(100000);
                  target[i]->target_position = target_int32[i];
               }
               else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY) // csv
               {
                  target[i]->target_velocity = target_int32[i];
               }
            }

            // target->target_velocity = (int32)(100000);
            // target->target_position = (int32)(2620039);
            for (i = 1; i <= 50000; i++)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               //  printf("Processdata cycle %4d, WKC %d,EWKC %d", i, wkc,expectedWKC);
               if (wkc >= expectedWKC)
               {

                  for (int j = 0; j < NUM_TARGET; j++)
                  {
                     printf("cycle %4d, WKC %d, motor %d", i, wkc, j);
                     printf("pos: %f, vel: %f, torque: %f pos err: %f, vel err: %f,  status: 0x%x, control: 0x%x",
                            (double)val[j]->position_actual / gear_ratio[j] * (2 * M_PI) / 131071,
                            (double)val[j]->velocity_actual / gear_ratio[j] * (2 * M_PI) / 131071,
                            (double)val[j]->torque_actual * 27720. / 1000000. * 2.,
                            (double)(target[j]->target_position - val[j]->position_actual) / gear_ratio[j] * (2 * M_PI) / 131071,
                            (double)(target[j]->target_velocity - val[j]->velocity_actual) / gear_ratio[j] * (2 * M_PI) / 131071,
                            val[j]->status_word, target[j]->control_word);

                     switch (target[j]->control_word)
                     {
                     case 0:
                        target[j]->control_word = 6;
                        break;
                     case 6:
                        target[j]->control_word = 7;
                        break;
                     case 7:
                        target[j]->control_word = 15;
                        break;
                     case 128:
                        target[j]->control_word = 0;
                        break;
                     default:
                        if (val[j]->status_word >> 3 & 0x01)
                        {
                           READ(1, 0x1001, 0, buf8, "Error");
                           target[j]->control_word = 128;
                        }
                     }
                     printf("  Target pos: %d, control: 0x%x", target[j]->target_position, target[j]->control_word);
                     printf("\n");
                  }

                  // //check error
                  // if ((val->status_word & 0x0fff) != 0x0237)
                  // {
                  //    CHECKERROR(1);
                  // }

                  // if (reachedInitial == 0  && (val->status_word & 0x0fff) == 0x0237) {
                  //       reachedInitial = 1; // motor reached target
                  // }

                  // // CheckSendFlag();

                  // if (reachedInitial == 0  && (val->status_word & 0x0fff) == 0x0237) {
                  //       // // target->target_velocity = (int32)(100);
                  //       // target->target_position = (int32)(2620000);
                  // }

                  needlf = TRUE;
               }
               osal_usleep(1000); // 1ms
            }
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExecute as root\n", ifname);
   }
}

void ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         // if(!ec_group[currentgroup].docheckstate)
         // printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

void parse_doubles(char* input, double output_array[NUM_TARGET]) {
    std::stringstream ss(input); 
    int count = 0;

    while (ss.good() && count < NUM_TARGET) { 
        std::string substr;
        getline(ss, substr, ',');

        double value;
        std::istringstream iss(substr);
        if (!(iss >> value)) {
            throw std::runtime_error("Invalid number format in input string");
        }

        output_array[count] = value; 
        count++;
    }
    // Initialize remaining elements to 0.0, if necessary
    for (int i = count; i < NUM_TARGET; ++i) {
        output_array[i] = 0.0;
    }
}

int main(int argc, char *argv[])
{  
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
   // get arguments from command line, contains the adapter,type of control mode: csp, csv,target position or target velocity,
   // if is csp mode, the second argument is the target position, if is csv mode, the second argument is the target velocity
   // if there is no argument, the program will print the available control mode and exit
   
   int8_t control_mode_int8;

   if (argc < 5)
   {
      // print argc and all argv
      printf("argc: %d\n", argc);
      for (int i = 0; i < argc; i++)
      {
         printf("argv[%d]: %s\n", i, argv[i]);
      }
      ec_adaptert *adapter = NULL;
      printf("Usage: simple_test ifname1 csp/csv target max_velocity gear\n");
      printf("\nAvailable adapters:\n");
      adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
      printf("control_mode: csp,csv\n");
      printf("target: target position if control mode is csp, target velocity if control mode is csv\n");

      printf("unit: target position is rad, target velocity is rad/s\n");
      return (0);
   }
   // else if the mode is csp,check the string, wheather is "csp", the target position is the third argument
   // else if the mode is csv,check the string, wheather is "csv", the target velocity is the third argument
   else
   {
      if (strcmp(argv[2], "csp") == 0)
      {  
         control_mode_int8 = CONTROL_MODE::CYCLIC_SYNC_POSITION;
         printf("Control mode: csp\n");
         // print the target position,which is double,unit is rad
         double target_position = atof(argv[3]);
         double max_velocity = atof(argv[4]);
         printf("Target position: %f rad\n", target_position);
         printf("Max velocity: %f rad/s \n", max_velocity);

      }
      else if (strcmp(argv[2], "csv") == 0)
      {  
         control_mode_int8 = CONTROL_MODE::CYCLIC_SYNC_VELOCITY;
         printf("Control mode: csv\n");
         // print the target velocity,which is double,unit is rad/s
         double target_velocity = atof(argv[3]);
         double max_velocity = atof(argv[4]);
         printf("Target velocity: %f rad/s\n", target_velocity);
         printf("Max velocity: %f rad/s \n", max_velocity);
      }
      else
      {
         printf("Invalid control mode\n");
         return (0);
      }
   }


   double target_input_in[NUM_TARGET];
    try {
        parse_doubles(argv[3], target_input_in);

        // Print results
        std::cout << "Output array: {";
        for (int i = 0; i < NUM_TARGET; ++i) {
            std::cout << target_input_in[i];
            if (i != NUM_TARGET - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "}\n";

    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Indicate failure
   }
   // return 0;

   osal_thread_create(&thread1, 128000, (void *)(&ecatcheck), NULL);

   simpletest(argv[1], control_mode_int8, target_input_in, atof(argv[4]));
   printf("End program\n");
   return (0);
}
//sudo ./simple_test enp3s0 csv 0.1,0.2,0.4,0.8,0.1,0.2 1
// sudo ./simple_test enp3s0 csp 0,0,0,0,0,0 1
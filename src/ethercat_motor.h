/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Boxi 2024 - 2025
 * (c)Arthur Ketels 2010 - 2011
 */

#ifndef _ETHERCAT_MOTOR_H
#define _ETHERCAT_MOTOR_H

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <vector>

#include <pthread.h>
#include <thread> // TODO

// #include <pybind11/pybind11.h>
// #include <pybind11/numpy.h> // For passing NumPy arrays

#include <iostream>
#include <sstream>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

#define NUM_TARGET 6 // num of motors
// #define NUM_TARGET 1 // num of motors

constexpr double gear_ratio[] = {20, 20, 20, 20, 10, 10}; // 6
// double gear_ratio[] = {1,1,1,1,1,1}; // hack changeback
constexpr double torque_multiplier[] = {1, 1, 1, 1, 1, 1}; // 6
// constexpr double torque_multiplier[] = {0, 0, 0, 0, 0, 1}; // 6


char IOmap[4096];

int expectedWKC;                    // Expected Working Counter value (used for data exchange validation).
volatile int wkc;                   // Actual Working Counter value received
boolean needlf;                     // Flag to control newline printing.
boolean inOP;                       // Flag indicating if slaves are in operational state.
uint8 currentgroup = 0;             // Currently active EtherCAT group.
boolean forceByteAlignment = FALSE; // Flag to control byte alignment of IO memory.

enum CONTROL_MODE : int8
{
    CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
    CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
    CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode
};

// 0x1607 RPDO8
struct out_ELMOt             // pdo
{                            // 1c12
    int32 target_position;   // 0x607a
    int32 target_velocity;   // 0x60ff
    int32 velocity_offset;   // 0x60b1
    int16 max_torque;        // 0x6072
    int16 target_torque;     // 0x6071 -1000~1000, 0.1% of "Motor rated current (0x6075)"
    int16 torque_offset;     // 0x60b2 -1000~1000, 0.1% of "Motor rated current (0x6075)"
    uint16 control_word;     // 0x6040 pdo domain 0x1604
    uint8 mode_of_operation; // 0x6060
};
// 0x1A07; // "TPDO8 Mapping"
struct in_ELMOt                   // pdo
{                                 // 1c13
    int32 position_actual;        // 0x6064
    int32 position_follow_err;    // 0x60f4
    int32 velocity_actual;        // 0x606C
    int16 torque_actual;          // 0x6077
    uint16 status_word;           // 0x6041
    uint8 mode_of_operation_disp; // 0x6061
};

#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

#define CHECKERROR(slaveId)                                                                                                                                                                       \
    {                                                                                                                                                                                             \
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
    /** set PDO mapping */
    uint16_t TxAddr = 0x1607; // "RPDO8 Mapping"
    uint16_t RxAddr = 0x1A07; // "TPDO8 Mapping"

    // 0 disable
    wkc += ec_SDOwrite(slave, TxAddr, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE);
    sdoObj = 0x607A0020; // Target position, INTEGER32 (0020)
    wkc += ec_SDOwrite(slave, TxAddr, 0x01, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60FF0020; // Target velocity, INTEGER32 (0020)
    wkc += ec_SDOwrite(slave, TxAddr, 0x02, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60B10020; // Velocity offset, INTEGER32 (0020)
    wkc += ec_SDOwrite(slave, TxAddr, 0x03, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60720010; // Max torque, UNSIGNED16 (0010)
    wkc += ec_SDOwrite(slave, TxAddr, 0x04, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60710010; // Target torque, INTEGER16 (0010) // current%
    wkc += ec_SDOwrite(slave, TxAddr, 0x05, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60b20010; // torque offset, INTEGER16 (0010)
    wkc += ec_SDOwrite(slave, TxAddr, 0x06, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60400010; // Controlword, UNSIGNED16 (0010)
    wkc += ec_SDOwrite(slave, TxAddr, 0x07, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60600008; // Modes of operation, INTEGER8 (0008)
    wkc += ec_SDOwrite(slave, TxAddr, 0x08, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    enable_bits = 0x08; //            0x00      "Number of sub indexes"  [UNSIGNED8        RWR_R_]      0x06 / 6
    wkc += ec_SDOwrite(slave, TxAddr, 0x00, FALSE, sizeof(enable_bits), &(enable_bits), EC_TIMEOUTSTATE);

    // 0 disable
    wkc += ec_SDOwrite(slave, RxAddr, 0x00, FALSE, sizeof(disable_bits), &(disable_bits), EC_TIMEOUTSTATE);
    sdoObj = 0x60640020; // Position actual value
    wkc += ec_SDOwrite(slave, RxAddr, 0x01, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60F40020; // Following error actual value
    wkc += ec_SDOwrite(slave, RxAddr, 0x02, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x606C0020; // velocity actual value
    wkc += ec_SDOwrite(slave, RxAddr, 0x03, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60770010; // Torque actual value
    wkc += ec_SDOwrite(slave, RxAddr, 0x04, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60410010; // Statusword
    wkc += ec_SDOwrite(slave, RxAddr, 0x05, FALSE, sizeof(sdoObj), &(sdoObj), EC_TIMEOUTSTATE);
    sdoObj = 0x60610008; // Modes of operation display
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

class Motor
{

public:
    std::array<int32_t, NUM_TARGET> target_int32{};

    std::array<int32_t, NUM_TARGET> target_offset{};

    uint32_t max_velocity_uint[NUM_TARGET];
    double target_input_rotor[NUM_TARGET]; // scaled by the gear ratio
    std::string ifname;                    // use ifconfig in commandline to find the ethernet name

    int8_t control_mode_int8;
    double max_velocity = 0;
    double max_torque = 0;
    struct out_ELMOt *target[NUM_TARGET];
    struct in_ELMOt *val[NUM_TARGET];

    bool should_print = true;

    // OSAL_THREAD_HANDLE thread1; // ecatcheck
    // OSAL_THREAD_HANDLE thread2;

    std::thread thread1;
    std::thread thread2;
    std::atomic<bool> should_terminate{false};

    // Motor()
    // {
    //    ifname = "enp3s0";
    //    control_mode_int8 = CONTROL_MODE::CYCLIC_SYNC_VELOCITY;
    //    max_velocity = 0.5;
    //    double target_input_in[] = {0.5, 0.5};

    //    for (int i = 0; i < NUM_TARGET; i++)
    //    {
    //       target_input_rotor[i] = target_input_in[i] * gear_ratio[i];
    //       max_velocity_uint[i] = (uint32_t)(max_velocity * gear_ratio[i] / 0.10472); // 1 RPM = 0.10472 rad/s
    //       // printf("%d %d \n ", i, max_velocity_uint[i]);
    //    }
    //    printf("reached 1\n");

    //    // osal_thread_create(&thread1, 128000, (void *)(&ecatcheck), NULL);
    // }
    
    Motor(const std::string ifname_, int8_t control_mode_int8_, std::vector<double> target_input_in, double max_velocity_, double max_torque_)

    Motor()
    {
        std::string ifname = "enp3s0";
        // std::vector<double> target_input_in{0, 0, 0, 0, 0, 0};
        double max_velocity = 0.5;
        double max_torque = 20;
        double control_mode_int8 = CONTROL_MODE::CYCLIC_SYNC_VELOCITY;
        _init(ifname, control_mode_int8, max_velocity, max_torque);
    }

    Motor(const std::string &ifname, int8_t control_mode_int8, double max_velocity, double max_torque)
    {
        _init(ifname, control_mode_int8, max_velocity, max_torque);
    }

    void _init(const std::string &ifname_, int8_t control_mode_int8_, double max_velocity_, double max_torque_)
    {
        ifname = ifname_;
        control_mode_int8 = control_mode_int8_;
        max_velocity = max_velocity_;
        max_torque = max_torque_;
        for (int i = 0; i < NUM_TARGET; i++)
        {
            max_velocity_uint[i] = (uint32_t)(max_velocity * gear_ratio[i] / 0.10472); // 1 RPM = 0.10472 rad/s
            printf("%d %d \n ", i, max_velocity_uint[i]);
        }
    }

    void set_target_input(std::vector<double> target_input_in)
    {
        if ((control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION) || control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY)
        {
            // the range of encoder is 0~2^17-1,unit is cnt (count)
            // the target position's unit is rad
            // convert the target position from rad to cnt
            for (int i = 0; i < NUM_TARGET; i++)
            {
                target_input_rotor[i] = (target_input_in[i] + target_offset[i]) * gear_ratio[i]; // rad
                target_int32[i] = (int32_t)(target_input_rotor[i] * 131072 / (2 * M_PI));
            }
        }
        else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_TORQUE)
        {
            for (int i = 0; i < NUM_TARGET; i++)
            {
                target_int32[i] = (int32_t)(target_input_rotor[i]); // NO offset applied
            }
        }
    }

    ~Motor()
    {
        should_terminate = true;
        // pthread_detach(*thread1);
        if (thread1.joinable())
        {
            thread1.join();
        }
        if (thread2.joinable())
        {
            thread2.join();
        }
        //    udp_server.close();
    }

    void set_should_terminate(bool value)
    {
        should_terminate = value;
        std::cout << "set_should_terminate()" << std::endl;
    }

    void set_target_offset(std::vector<double> target_offset)
    {
        for (int i = 0; i < NUM_TARGET; i++)
        {
            // target_input_rotor[i] += target_offset[i] * gear_ratio[i]; // rad
            this->target_offset[i] = target_offset[i];
        }
    }

    void run()
    {
        // ecatcheck(nullptr); // works
        // _run();
        thread1 = std::thread(&Motor::ecatcheck, this, nullptr);
        // thread2 = std::thread(&Motor::_run, this);


        // osal_thread_create(&thread1, 128000, (void *)(&ecatcheck), NULL);
        _run();
    }

    void _run_loop()
    {

        /* cyclic loop for slaves*/

        for (size_t i = 0; i < NUM_TARGET; i++)
        {
            // SET INITAL GOAL
            // target[i]->control_word = 0; // stop
            target[i]->max_torque = (uint16)(max_torque * torque_multiplier[i]);
            if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION) // csp
            {
                // READ(1, 0x6080, 0, buf32, "*max velocity*");
                // WRITE(1, 0x6080, 0, buf32, 100000, "*max velocity*"); usleep(100000);
                target[i]->target_position = target_int32[i];
            }
            else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY) // csv
            {
                target[i]->target_velocity = target_int32[i];
                // target[i]->velocity_offset = 10000;//example of adding velocity offset
            }
            // else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_TORQUE) // csv
            // {
            //    target[i]->torque_offset = 30;
            // }
        }

        // target->target_velocity = (int32)(100000);
        // target->target_position = (int32)(2620039);

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        //  printf("Processdata cycle %4d, WKC %d,EWKC %d", i, wkc,expectedWKC);
        if (wkc >= expectedWKC)
        {
            if (should_print)
            {
                printf("\npos:");
                for (int j = 0; j < NUM_TARGET; j++)
                {
                    printf("%8.5f,", (double)(val[j]->position_actual) / gear_ratio[j] * (2 * M_PI) / 131072);
                }
                printf("\nprr:"); // posotion error
                for (int j = 0; j < NUM_TARGET; j++)
                {
                    printf("%8.5f,", (double)(val[j]->position_follow_err) / gear_ratio[j] * (2 * M_PI) / 131072);
                }
                printf("\nvel:");
                for (int j = 0; j < NUM_TARGET; j++)
                {
                    printf("%8.5f,", (double)(val[j]->velocity_actual) / gear_ratio[j] * (2 * M_PI) / 131072);
                }
                printf("\ntor:"); // torque raw
                for (int j = 0; j < NUM_TARGET; j++)
                {
                    printf("%8d,", val[j]->torque_actual);
                }
            }

            for (int j = 0; j < NUM_TARGET; j++)
            {
                // printf("cycle %4d, WKC %d, motor %d ", i, wkc, j);
                // printf("pos: %8.5f, pos err: %8.5f, vel: %7.3f, torque: %7.3f , status_word: %x, op_mode: %x \n",
                //        (double)(val[j]->position_actual) / gear_ratio[j] * (2 * M_PI) / 131072,
                //        (double)(val[j]->position_follow_err) / gear_ratio[j] * (2 * M_PI) / 131072,
                //        (double)(val[j]->velocity_actual) / gear_ratio[j] * (2 * M_PI) / 131072,
                //        (double)val[j]->torque_actual * 27720. / 1000000. * 2.,
                //        val[j]->status_word,
                //        val[j]->mode_of_operation_disp);
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
                        uint8 buf8;
                        // READ(1, 0x1001, 0, buf8, "Error");
                        target[j]->control_word = 128;
                    }
                }
                // printf("  Target pos: %d, control: 0x%x", target[j]->target_position, target[j]->control_word);
                // printf("\n");
            }

            // //check error
            // if ((val->status_word & 0x0fff) != 0x0237)
            // {
            //    CHECKERROR(1);
            // }

            // if (reachedInitial == 0  && (val->status_word & 0x0fff) == 0x0237) {
            //       reachedInitial = 1; // motor reached target
            // }

            // if (reachedInitial == 0  && (val->status_word & 0x0fff) == 0x0237) {
            //       // // target->target_velocity = (int32)(100);
            //       // target->target_position = (int32)(2620000);
            // }

            needlf = TRUE;
        }
        /* The cycle times in CSP mode with 2^n * 125 µs (for n =1 to 8) are:
           250 µs, 500 µs, 1 ms, 2 ms, 4 ms, 8 ms, 16 ms or 32 ms.
           in reality, the motor only accept < 1600 us
        */
        osal_usleep(1000); // 1ms
    }
    void _run()
    {

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
        if (ec_init(ifname.c_str()))
        {
            printf("ec_init on %s succeeded.\n", ifname.c_str());
            /* find and auto-config slaves */
            for (size_t i = 0; i < NUM_TARGET; i++)
            {
                ec_slave[1 + i].PO2SOconfigx = &ELMOsetupGOLD;
            }

            if (ec_config_init(FALSE) > 0)
            {

                ec_configdc();
                // usleep(100000);
                // usleep(10000);
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ELMOsetupGOLD(&ecx_context, i) != 25)
                    {
                        printf("error: ELMOsetupGOLD \n");
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
                printf("Slaves mapped, state to SAFE_OP.\n");

                // /** disable heartbeat alarm */
                // for (int i = 1; i <= ec_slavecount; i++) {
                //     READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                //     WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                //     WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                //     WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
                // }

                for (int i = 0; i < ec_slavecount; i++)
                {
                    READ(i + 1, 0x6075, 0, buf32, "Motor Rated Current [in mA]");
                    READ(i + 1, 0x6076, 0, buf32, "Motor Rated torque [in mA]"); // same as the motor rated current

                    READ(i + 1, 0x1c12, 0, buf32, "rxPDO:0");
                    READ(i + 1, 0x1c13, 0, buf32, "txPDO:0");

                    READ(i + 1, 0x1c12, 1, buf32, "rxPDO:1");
                    READ(i + 1, 0x1c13, 1, buf32, "txPDO:1");

                    READ(i + 1, 0x6064, 0, sbuf32, "*position actual value*");

                    WRITE(i + 1, 0x6060, 0, buf8, control_mode_int8, "OpMode");
                    READ(i + 1, 0x6061, 0, buf8, "OpMode display");
                    WRITE(i + 1, 0x6080, 0, buf32, max_velocity_uint[i], "*Max motor speed*");
                    WRITE(i + 1, 0x6072, 0, buf16, (uint16)(max_torque * torque_multiplier[i]), "*Maximal torque*"); // per thousand of rated torque

                    // READ(i + 1, 0x1604, 0, buf32, "rxPDO:0 1604");
                    // READ(i + 1, 0x1a07, 0, buf32, "txPDO:0 0x1a07");
                    // READ(i + 1, 0x1604, 1, buf32, "rxPDO:1 1604");
                    // READ(i + 1, 0x1a07, 1, buf32, "txPDO:1 0x1a07");
                    // READ(i + 1, 0x1604, 2, buf32, "rxPDO:2 1604");
                    // READ(i + 1, 0x1a07, 2, buf32, "txPDO:2 0x1a07");
                    // READ(i + 1, 0x1604, 3, buf32, "rxPDO:3 1604");
                    // READ(i + 1, 0x1a07, 3, buf32, "txPDO:3 0x1a07");
                    // READ(i + 1, 0x1604, 4, buf32, "rxPDO:4 1604");
                    // READ(i + 1, 0x1a07, 4, buf32, "txPDO:4 0x1a07");
                }

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

                    // /**
                    //  * Drive state machine transistions
                    //  *   0 -> 6 -> 7 -> 15
                    //  */
                    int sleep_us = 10000;
                    for (int i = 1; i <= ec_slavecount; i++) // 0 is broadcast
                    {
                        if (buf16 == 0x218)
                        {
                            WRITE(i, 0x6040, 0, buf16, 128, "*control word*");
                        }
                    }
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        WRITE(i, 0x6040, 0, buf16, 0, "*control word*");
                    }
                    usleep(sleep_us);
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        WRITE(i, 0x6040, 0, buf16, 6, "*control word*");
                    }
                    usleep(sleep_us);
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        WRITE(i, 0x6040, 0, buf16, 7, "*control word*");
                    }
                    usleep(sleep_us);
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); // Fault reset
                    }
                    usleep(sleep_us);

                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        CHECKERROR(i);
                        READ(i, 0x1001, 0, buf8, "Error");
                    }

                    // int reachedInitial = 0; //TODO CHAGE TO ARRAY FOR EACH MOTOR, THIS IS NOT USED

                    // auto start = std::chrono::high_resolution_clock::now(); // timming code

                    // for (i = 1; i <= 10000; i++) // TODO CHANGE
                    while (!should_terminate)
                    {
                        _run_loop();
                    }

                    // // // timming code: Get the end time
                    // auto end = std::chrono::high_resolution_clock::now();
                    // // Calculate the elapsed time
                    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    // // Print the elapsed time
                    // std::cout << "Elapsed time: " << elapsed.count() << " milliseconds" << std::endl;

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
            printf("No socket connection on %s\nExecute as root\n", ifname.c_str());
        }
    }

    void ecatcheck(void *ptr)
    {
        int slave;
        (void)ptr; /* Not used */

        while (!should_terminate)
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
};

#endif /* _ETHERCAT_MOTOR_H */
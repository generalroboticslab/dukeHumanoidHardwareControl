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

// #include <stdio.h>

// #include <sys/time.h>
// #include <unistd.h>
#include <math.h>
#include <atomic>
#include <pthread.h>
#include <thread> // TODO

// #include <pybind11/pybind11.h>
// #include <pybind11/numpy.h> // For passing NumPy arrays

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>
// #include <Eigen/Core>

#include "ethercat.h"

template <typename ScalarType, int NumElements>
using VectorNd = Eigen::Matrix<ScalarType, NumElements, 1>;

#define EC_TIMEOUTMON 500

#define NUM_TARGET 10 // num of motors

const VectorNd<double, 10> gear_ratio_all = {18, 20, 18, 18, 10,18, 20, 18, 18, 10}; // hack max 6 motors
const VectorNd<double, 10> torque_multiplier_all = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// 2020-20 motor current torue ratio: toruqe [NM] = 2.0 * current [A] (from measurement)
// 2010-10 motor current torque ratio: toruqe [NM] = 1.6325 * current [A] (from figure)

enum CONTROL_MODE : int8
{
    CYCLIC_SYNC_POSITION = 8, // Cyclic sync position mode
    CYCLIC_SYNC_VELOCITY = 9, // Cyclic sync velocity mode
    CYCLIC_SYNC_TORQUE = 10   // Cyclic sync torque mode
};

// 0x1607 RPDO8
struct tx_pdo_t              // tx pdo
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
struct rx_pdo_t                   // rx pdo
{                                 // 1c13
    int32 position_actual;        // 0x6064
    int32 position_follow_err;    // 0x60f4
    int32 velocity_actual;        // 0x606C
    int16 torque_actual;          // 0x6077
    uint16 status_word;           // 0x6041
    uint8 mode_of_operation_disp; // 0x6061
};

template <typename T>
void print_eigen_vec(T vector, const char format[], const char pre[], const char after[] = "\n")
{
    printf("%s", pre);
    for (auto value : vector)
    {
        printf(format, value);
    }
    printf("%s", after);
}

template <typename T>
int READ(int slaveId, int idx, int sub, T &buf, const std::string &comment, bool verbose = true)
{ // blocking, takes about 2ms
    buf = 0;
    int __s = sizeof(buf);
    int wkc = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);
    if (verbose) {
    // printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, wkc, __s, (unsigned int)buf, (unsigned int)buf, comment.c_str());
    // ANSI Code based on wkc, print in red if wkc==0
    const char *colorCode = (wkc == 0) ? "\033[31;1m" : "";
    printf("%sSlave:%2d - READ  at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\033[0m\n",
           colorCode, slaveId, idx, sub, wkc, __s, (unsigned int)buf, (unsigned int)buf, comment.c_str());
    }
    return wkc;
}

template <typename T>
int WRITE(int slaveId, int idx, int sub, T &buf, int value, const std::string &comment , bool verbose = true)
{ // blocking, takes about 2ms
    buf = value;
    int __s = sizeof(buf);
    int wkc = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);
    if (verbose){
    // printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t{%s}\n", slaveId, idx, sub, wkc, __s, (unsigned int)buf, (unsigned int)buf, comment.c_str());
    // ANSI Code based on wkc, print in red if wkc==0
    const char *colorCode = (wkc == 0) ? "\033[31;1m" : "";
    printf("%sSlave:%2d - WRITE at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\033[0m\n",
           colorCode, slaveId, idx, sub, wkc, __s, (unsigned int)buf, (unsigned int)buf, comment.c_str());
    }
    return wkc;
}

void CHECKERROR(int slaveId)
{
    ecx_readstate(&ecx_context);
    printf("EC> \"%s\" %x - %x [%s] \n", (char *)ecx_elist2string(&ecx_context), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));
}

int ELMOsetupGOLD(ecx_contextt *context, uint16 slave)
{
    int wkc = 0;
    uint32_t sdoObj = 0x00000000;
    uint8_t disable_bits = 0x00;
    uint8_t enable_bits = 0x01;
    uint16_t objAddr = 0x0000;
    /** set PDO mapping */
    uint16_t TxAddr = 0x1607; // "RPDO8 Mapping", relative to slave it is rx
    uint16_t RxAddr = 0x1A07; // "TPDO8 Mapping", relative to slave it is tx

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

    int8 op_mode = CONTROL_MODE::CYCLIC_SYNC_VELOCITY; // 8:position 9:velocity 10:torque
    // de0x6060      "Modes of operation"
    wkc += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(op_mode), &op_mode, EC_TIMEOUTSTATE); //    cyclic sychronous position mode
    // printf("supported drive modes: %d\n", op_mode);
    printf("ELMOsetupGOLD(): slave:%d wkc: %d\n", slave, wkc);

    return wkc;
}

class Motor
{
public:
    bool debug = true;
    bool should_print = true;

    VectorNd<double, NUM_TARGET> target_position = VectorNd<double, NUM_TARGET>::Zero();        // target position [rad]
    VectorNd<double, NUM_TARGET> target_position_offset = VectorNd<double, NUM_TARGET>::Zero(); // target position offset [rad]
    VectorNd<int32, NUM_TARGET> target_position_int32 = VectorNd<int32, NUM_TARGET>::Zero();

    VectorNd<double, NUM_TARGET> target_velocity = VectorNd<double, NUM_TARGET>::Zero(); // target velocity [rad/s]
    VectorNd<double, NUM_TARGET> velocity_offset = VectorNd<double, NUM_TARGET>::Zero(); // feed-forward velocity offset [rad/s]
    VectorNd<int32, NUM_TARGET> target_velocity_int32 = VectorNd<int32, NUM_TARGET>::Zero();
    VectorNd<int32, NUM_TARGET> velocity_offset_int32 = VectorNd<int32, NUM_TARGET>::Zero(); // feed-forward velocity offset in the encoder space [cnt/s]

    VectorNd<double, NUM_TARGET> target_torque_raw = VectorNd<double, NUM_TARGET>::Zero(); // target toruqe (raw) as of 0.1% rated torque
    VectorNd<int16, NUM_TARGET> target_torque_int16 = VectorNd<int16, NUM_TARGET>::Zero(); // target torque as of 0.1% rated torque
    VectorNd<int16, NUM_TARGET> torque_offset_int16 = VectorNd<int16, NUM_TARGET>::Zero(); // feed-forward torque offset as of 0.1% rated torque

    VectorNd<uint32, NUM_TARGET> max_velocity_uint = VectorNd<uint32, NUM_TARGET>::Zero();

    VectorNd<double, NUM_TARGET> actual_position = VectorNd<double, NUM_TARGET>::Zero();       // actual position [rad]
    VectorNd<double, NUM_TARGET> actual_position_error = VectorNd<double, NUM_TARGET>::Zero(); // actual position error [rad]
    VectorNd<double, NUM_TARGET> actual_velocity = VectorNd<double, NUM_TARGET>::Zero();       // actual velocity [rad/s]

    VectorNd<double, NUM_TARGET> actual_torque_raw = VectorNd<double, NUM_TARGET>::Zero(); // actual torque (raw) as of 0.1% rated torque
    VectorNd<double, NUM_TARGET> actual_torque = VectorNd<double, NUM_TARGET>::Zero();     // actual torque in [A] actual_torque=actual_torque_raw*rated_torque*1e-3

    VectorNd<double, NUM_TARGET> rated_torque = VectorNd<double, NUM_TARGET>::Zero(); // actual velocity (raw) in rated current [A]

    VectorNd<uint16, NUM_TARGET> status_word = VectorNd<uint16, NUM_TARGET>::Zero(); // status word

    // gear_ratio_all and torque_multiplier_all may have more elements (for debug)
    VectorNd<double, NUM_TARGET> gear_ratio = gear_ratio_all.head(NUM_TARGET);
    VectorNd<double, NUM_TARGET> torque_multiplier = torque_multiplier_all.head(NUM_TARGET);

    VectorNd<double, NUM_TARGET> max_torque = VectorNd<double, NUM_TARGET>::Zero();

    std::string ifname; // use ifconfig in commandline to find the ethernet name

    int8_t control_mode_int8;
    double max_velocity = 0;


    // double max_torque = 0;
    struct tx_pdo_t *target[NUM_TARGET];
    struct rx_pdo_t *val[NUM_TARGET];

    // OSAL_THREAD_HANDLE thread1; // ecatcheck
    // OSAL_THREAD_HANDLE thread2;

    std::thread thread1; // ecatcheck
    std::thread thread2;

    std::atomic<bool> should_terminate{false};

    bool init_successful = false;

    Motor()
    {
        std::string ifname = "enp3s0";
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
        for (int i = 0; i < NUM_TARGET; i++)
        {
            max_torque(i) = max_torque_;
            max_velocity_uint(i) = (uint32_t)(max_velocity * gear_ratio(i) / 0.10472); // 1 RPM = 0.10472 rad/s
        }
        print_eigen_vec(max_velocity_uint, "%u ", "max_velocity_uint: ");
        configure();
    }

    void set_max_torque(const Eigen::Ref<Eigen::VectorXd> _max_torque)
    {
        max_torque = _max_torque;
    }

    void set_target_position(const Eigen::Ref<Eigen::VectorXd> _target_position)
    {
        target_position = _target_position;
        // the range of encoder is 0~2^17-1,unit is cnt (count)
        // the target position's unit is rad
        // convert the target position from rad to cnt
        // scaled by the gear ratio
        target_position_int32 = (gear_ratio.cwiseProduct(target_position + target_position_offset) * 131072 / (2 * M_PI)).cast<int32_t>();
    }

    void set_target_position_offset(const Eigen::Ref<Eigen::VectorXd> _target_position_offset)
    {
        target_position_offset = _target_position_offset;
        target_position_int32 = (gear_ratio.cwiseProduct(target_position + target_position_offset) * 131072 / (2 * M_PI)).cast<int32_t>();
    }

    void set_target_velocity(const Eigen::Ref<Eigen::VectorXd> _target_velocity)
    {
        target_velocity = _target_velocity;
        // the range of encoder is 0~2^17-1,unit is cnt (count)
        // the target velocity's unit is rad/s
        // convert the target velocity from rad to cnt/s
        // scaled by the gear ratio
        target_velocity_int32 = (gear_ratio.cwiseProduct(target_velocity) * 131072 / (2 * M_PI)).cast<int32_t>();
        // print_eigen_vec(target_velocity_int32, "%d ", "(set_target_velocity()) target_velocity_int32: ");
    }

    void set_velocity_offset(const Eigen::Ref<Eigen::VectorXd> _velocity_offset)
    {
        velocity_offset = _velocity_offset;
        velocity_offset_int32 = (gear_ratio.cwiseProduct(velocity_offset) * 131072 / (2 * M_PI)).cast<int32_t>();
        std::cout << "Velocity offset: " << velocity_offset.transpose() << ", Gear ratio: " << gear_ratio.transpose() << ", PI value: " << M_PI << std::endl;

    }

    void set_target_torque(const Eigen::Ref<Eigen::VectorXd> _target_torque)
    {
        target_torque_raw = _target_torque;
        target_torque_int16 = target_torque_raw.cast<int16>(); // NO offset applied //TODO CHANGE TO INT16
    }

    void set_torque_offset(const Eigen::Ref<Eigen::VectorXd> _torque_offset)
    {
        torque_offset_int16 = _torque_offset.cast<int16>(); // NO offset applied
    }

    void set_target_input(const Eigen::Ref<Eigen::VectorXd> target_input)
    {
        // Check the size
        if (target_input.size() != NUM_TARGET)
        {
            throw std::runtime_error("Input must be exactly " + std::to_string(NUM_TARGET) + "size");
        }
        if ((control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION))
        {
            set_target_position(target_input);
        }
        else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY)
        {
            set_target_velocity(target_input);
        }
        else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_TORQUE)
        {
            set_target_torque(target_input);
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

    void set_should_terminate(const bool value)
    {
        should_terminate = value;
        printf("set_should_terminate()\n");
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

    // configure motor
    int configure()
    {

        printf("Starting ethercat motor\n");
        /* initialise SOEM, bind socket to ifname */
        if (ec_init(ifname.c_str()) <= 0)
        {
            printf("\033[31;1mNo socket connection on %s! Execute as root (use 'sudo')\033[0m\n", ifname.c_str());
            return -1;
        }
        // ec_init successful
        printf("ec_init on %s succeeded.\n", ifname.c_str());

        /* find and auto-config slaves */
        for (int i = 0; i < NUM_TARGET; i++)
        {
            ec_slave[1 + i].PO2SOconfigx = &ELMOsetupGOLD;
        }
        if (ecx_config_init(&ecx_context, false) <= 0)
        {
            printf("\033[31;1mNo slaves found!\n"
                   "check if the slave is powered on or try resetting the adaptor with:\n"
                   "sudo ip link set %s down\nsudo ip link set %s up\n\033[0m\n",
                   ifname.c_str(), ifname.c_str());
            return -1;
        }

        // ecx_config_init successful
        printf("ecx_config_init succeeded.\n");

        ecx_configdc(&ecx_context);
        // osal_usleep(100000);
        // osal_usleep(10000);
        for (int i = 1; i <= ec_slavecount; i++)
        {
            int wkc = ELMOsetupGOLD(&ecx_context, i);
            int expected_wkc=25;
            if (wkc != expected_wkc)
            {
                printf("\033[31merror: ELMOsetupGOLD\n expected_wkc=%d, actual wkc=%d\033[39m \n",expected_wkc,wkc); // TODO add expeted wkc
                        // exit(1);
            };
        }


        if (forceByteAlignment)
        {
            ecx_config_map_group(&ecx_context, &IOmap, 0);
            // ec_config_map_aligned(&IOmap);
        }
        else
        {
            ecx_config_map_group_aligned(&ecx_context, &IOmap, 0);
            // ec_config_map(&IOmap);
        }
        // ELMOsetupGOLD successful
        printf("%d slaves found and configured.\n", ec_slavecount);


        // show slave info
        if(should_print){
            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                    i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                    ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }
        }

        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
        printf("Slaves mapped, state to SAFE_OP.\n");

        // /** disable heartbeat alarm */
        // for (int i = 1; i <= ec_slavecount; i++) {
        //     uint32 buf32;
        //     uint8 buf8;
        //     uint16 buf16;
        //     READ<uint32>(i, 0x10F1, 2, buf32, "Heartbeat?");
        //     WRITE<uint32>(i, 0x10F1, 2, buf32, 1, "Heartbeat");

        //     WRITE<uint8>(i, 0x60c2, 1, buf8, 2, "Time period");
        //     WRITE<uint16>(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
        // }

        for (int i = 0; i < ec_slavecount; i++)
        {
            // READ<uint32>(i + 1, 0x6075, 0, buf32, "Motor Rated Current [in mA]");

            READ<uint32>(i + 1, 0x6076, 0, buf32, "Motor Rated torque [in mA]", should_print); // same as the motor rated current
            rated_torque(i) = ((double)buf32)*1e-3; // 1mA = 1e-3A

            // READ<uint32>(i + 1, 0x1c12, 0, buf32, "rxPDO:0");
            // READ<uint32>(i + 1, 0x1c12, 1, buf32, "rxPDO:1");

            // READ<uint32>(i + 1, 0x1c13, 0, buf32, "txPDO:0");
            // READ<uint32>(i + 1, 0x1c13, 1, buf32, "txPDO:1");

            READ<uint32>(i + 1, 0x6079, 0, buf32, "DC link circuit voltage", should_print);

            READ<int32>(i + 1, 0x6064, 0, sbuf32, "*position actual value*", should_print);
            actual_position(i) = (double)(sbuf32) / gear_ratio(i) * (2 * M_PI) / 131072;

            // WRITE<int8>(i + 1, 0x6060, 0, sbuf8, control_mode_int8, "OpMode");
            // READ<int8>(i + 1, 0x6061, 0, sbuf8, "OpMode display");

            WRITE<uint32>(i + 1, 0x6080, 0, buf32, max_velocity_uint(i), "*Max motor speed*",should_print);
            // READ<uint32>(i + 1, 0x6080, 0, buf32, "*Max motor speed*");

            WRITE<uint16>(i + 1, 0x6072, 0, buf16, (uint16)(max_torque(i) * torque_multiplier(i)), "*Maximal torque*",should_print); // per thousand of rated torque
            // READ<uint16>(i + 1, 0x6072, 0, buf16, "*Maximal torque*");                                               // per thousand of rated torque

            //     READ<uint32>(i + 1, 0x6065, 0, buf32, "Position following error window");
            //     READ<uint16>(i + 1, 0x6066, 0, buf16, "Position following error time out [ms]");
            //     READ<uint32>(i + 1, 0x6066, 0, buf32, "Position window");
            //     READ<uint32>(i + 1, 0x6067, 0, buf32, "Position window times [ms]");
        }
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        printf("Calculated expected workcounter %d\n", expectedWKC);

        for (size_t i = 0; i < NUM_TARGET; i++)
        {
            target[i] = (struct tx_pdo_t *)(ec_slave[1 + i].outputs);
            val[i] = (struct rx_pdo_t *)(ec_slave[1 + i].inputs);

            //  * Drive state machine transistions
            //  *   0 -> 6 -> 7 -> 15
            target[i]->control_word = 0;
            target[i]->mode_of_operation = control_mode_int8;

            target[i]->target_position = 0;
            target[i]->target_velocity = 0;
            target[i]->velocity_offset = 0;
            target[i]->target_torque = 0;
            target[i]->torque_offset = 0;
        }

        // set init_successful to true
        init_successful = true;
        return 0;
    }

    void _run()
    {
        if (init_successful)
        {
            // printf("exit here !!!!\n"); //HACK
            // exit(1);
            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            int chk = 200;
            /* wait for all slaves to reach OP state */ // takes about 3 ms
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state != EC_STATE_OPERATIONAL)
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            else // if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                // if (debug)
                // {
                //     printf("\ntarget[i]->control_word (hex): ");
                //     for (size_t i = 0; i < NUM_TARGET; i++)
                //     {
                //         printf("%X ", target[i]->control_word);
                //     }
                //     printf("\nval[i]->status_word (hex): ");
                //     for (size_t i = 0; i < NUM_TARGET; i++)
                //     {
                //         printf("%X ", val[i]->status_word);
                //     }
                //     printf("\n");
                // }

                int sleep_us = 10000;

                for (int i = 1; i <= ec_slavecount; i++)
                {
                    CHECKERROR(i);
                    // READ<uint8>(i, 0x1001, 0, buf8, "Error Register");
                }

                // int reachedInitial = 0; //TODO CHAGE TO ARRAY FOR EACH MOTOR, THIS IS NOT USED

                // auto start = std::chrono::high_resolution_clock::now(); // timming code

                // for (i = 1; i <= 10000; i++) // TODO CHANGE
                while (!should_terminate)
                {
                    _run_loop();
                    loop_counter++;
                }

                // // // timming code: Get the end time
                // auto end = std::chrono::high_resolution_clock::now();
                // // Calculate the elapsed time
                // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                // // Print the elapsed time
                // std::cout << "Elapsed time: " << elapsed.count() << " milliseconds" << std::endl;

                inOP = FALSE;
            }

            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }

        // exit gracefully
        printf("End ethercat motor, close socket\n");
        /* stop SOEM, close socket */
        ec_close();

        should_terminate = true;
    }

    void _run_loop()
    {
        /*
                Statusword       PDS FSA state
                ....````....````
                xxxxxxxxx0xx0000 Not ready to switch on
        mask:   0000000001001111 (79)
        result: 0000000000000000 (0)

                xxxxxxxxx1xx0000 Switch on disabled
        mask:   0000000001001111 (79)
        result: 0000000001000000 (64)

                xxxxxxxxx0xx1111 Fault reaction active
        mask:   0000000001001111 (79)
        result: 0000000000001111 (15)

                xxxxxxxxx0xx1000 Fault
        mask:   0000000001001111 (79)
        result: 0000000000001000 (8)

                xxxxxxxxx01x0001 Ready to switch on
        mask:   0000000001101111 (111)
        result: 0000000000100001 (33)

                xxxxxxxxx01x0011 Switched on
        mask:   0000000001101111 (111)
        result: 0000000000100011 (35)

                xxxxxxxxx01x0111 Operation enabled
        mask:   0000000001101111 (111)
        result: 0000000000100111 (39)

                xxxxxxxxx00x0111 Quick stop active
        mask:   0000000001101111 (111)
        result: 0000000000000111 (7)
        */
        /* cyclic loop for slaves*/
        for (int i = 0; i < NUM_TARGET; i++)
        {
            if (val[i]->status_word != status_word(i))
            {
                status_word(i) = val[i]->status_word;

                switch (val[i]->status_word & 79)
                {
                case 0: // Not ready to switch on
                    /* code */
                    printf("motor[%d] status_word=0x%-4X Not ready to switch on.\n", i, val[i]->status_word);
                    break;
                case 64:                         // Switch on disabled
                    target[i]->control_word = 6; // 0b0110 shut down
                    printf("motor[%d] status_word=0x%-4X Switch on disabled. sending control_word=0x%-4X\n", i, val[i]->status_word, target[i]->control_word);
                    break;
                case 15: // Fault reaction active
                    printf("motor[%d] status_word=0x%-4X Fault reaction active\n", i, val[i]->status_word);
                    break;
                case 8: // Fault
                    uint8 buf8; // HACK TODO change back 
                    uint16 buf16;
                    uint32 buf32;
                    READ<uint8>(1+i, 0x1001, 0, buf8, "Error 0x1001");

                    READ<uint8>(1+i, 0x1003, 0, buf8, "Error code 0x1003");

                    for (uint8 j = 1; j < buf8+1; j++) {
                        READ<uint32>(1+i, 0x1003, j, buf32, "Error code 0x1003");
                    }

                    READ<uint16>(1+i, 0x603F, 0, buf16, "Error code 0x603F");
                    int16 buf_int16;
                    READ<int16>(1+i, 0x605A, 0, buf_int16, "Quick stop option code 0x605A");


                    target[i]->control_word = 128;
                    printf("motor[%d] status_word=0x%-4X Fault. sending control_word=0x%-4X\n", i, val[i]->status_word, target[i]->control_word);
                    
                    
                    // exit(1);
                    break;

                default:
                    break;
                }
                switch (val[i]->status_word & 111)
                {
                case 33:                         // Ready to switch on
                    target[i]->control_word = 7; // 0b0111 switch on
                    printf("motor[%d] status_word=0x%-4X Ready to switch on. sending control_word=0x%-4X\n", i, val[i]->status_word, target[i]->control_word);
                    break;
                case 35:                          // Switched on
                    target[i]->control_word = 15; // 0b1111 switch_on+enable_operation: switch_on=1, enable_voltage=1, quick_stop=1, enable_operation=1, fault_reset=0,halt=0
                    printf("motor[%d] status_word=0x%-4X Switched on. sending control_word=0x%-4X\n", i, val[i]->status_word, target[i]->control_word);
                    break;
                case 39: // Operation enabled
                    printf("motor[%d] status_word=0x%-4X Operation enabled\n", i, val[i]->status_word);
                    break;
                case 7: // Quick stop active
                    printf("motor[%d] status_word=0x%-4X Quick stop active\n", i, val[i]->status_word);
                    break;
                default:
                    break;
                }
            }
        }

        // // disable_voltage (0) -> shut_down (6) -> 7 (switch_on) -> 15 (switch_on+enable_operation)
        // for (int i = 0; i < NUM_TARGET; i++)
        // {
        //     switch (target[i]->control_word & 0b10000000)
        //     {
        //     case 0:                          // 0: switch_on=0, enable_voltage=0, quick_stop=0, enable_operation=0,fault_reset=0,halt=0
        //         target[i]->control_word = 6; // 0b0110 shut down
        //         break;
        //     case 6:                          // 6: 0b0110  switch_on=0, enable_voltage=1, quick_stop=1, enable_operation=0,fault_reset=0,halt=0
        //         target[i]->control_word = 7; // 0b0111 switch on
        //         break;
        //     case 7:                           // 7: 0b0111  switch_on=1, enable_voltage=1, quick_stop=1, enable_operation=0,fault_reset=0,halt=0
        //         target[i]->control_word = 15; // 0b1111 switch_on+enable_operation: switch_on=1, enable_voltage=1, quick_stop=1, enable_operation=1, fault_reset=0,halt=0
        //         break;
        //     case 128:                        // fault reset
        //         target[i]->control_word = 0; // 0b0000
        //         break;
        //     default:
        //         if (val[i]->status_word >> 3 & 0x01)
        //         {
        //             uint8 buf8;
        //             READ<uint8>(1, 0x1001, 0, buf8, "Error");
        //             target[i]->control_word = 128;
        //         }
        //     }
        // }

        for (size_t i = 0; i < NUM_TARGET; i++)
        {
            target[i]->mode_of_operation = control_mode_int8; // TODO check why needs input again..
            target[i]->max_torque = (uint16)(max_torque(i) * torque_multiplier(i));
            if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_POSITION) // csp
            {
                // uint32 buf32;
                // READ<uint32>(1, 0x6080, 0,buf32, "*max velocity*");
                // WRITE<uint32>(1, 0x6080, 0, buf32, 100000, "*max velocity*");
                target[i]->target_position = target_position_int32(i);
                // target[i]->target_velocity = 0;
                target[i]->velocity_offset = velocity_offset_int32(i);
                // target[i]->target_torque = 0;
                target[i]->torque_offset = torque_offset_int16(i);
            }
            else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_VELOCITY) // csv
            {
                // target[i]->target_position = 0;
                target[i]->target_velocity = target_velocity_int32(i);
                // target[i]->target_torque = 0;
                target[i]->velocity_offset = velocity_offset_int32(i);
                target[i]->torque_offset = torque_offset_int16(i);
            }
            else if (control_mode_int8 == CONTROL_MODE::CYCLIC_SYNC_TORQUE) // csv
            {
                // target[i]->target_position = 0;
                // target[i]->target_velocity = 0;
                target[i]->target_torque = target_torque_int16(i);
                target[i]->torque_offset = torque_offset_int16(i);
            }
        }

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // printf("Processdata cycle %4d, WKC %d,EWKC %d", i, wkc,expectedWKC);
        // printf("\nWKC %d,EWKC %d", wkc, expectedWKC);
        if (wkc >= expectedWKC)
        {
            for (int i = 0; i < NUM_TARGET; i++)
            {
                actual_position(i) = (double)(val[i]->position_actual) / gear_ratio(i) * (2 * M_PI) / 131072;
                actual_position_error(i) = (double)(val[i]->position_follow_err) / gear_ratio(i) * (2 * M_PI) / 131072;
                actual_velocity(i) = (double)(val[i]->velocity_actual) / gear_ratio(i) * (2 * M_PI) / 131072;
                actual_torque_raw(i) = (double)(val[i]->torque_actual);
            }
            actual_torque = actual_torque_raw.cwiseProduct(rated_torque) * 1e-3;

            // if (debug)
            // {
            //     if (loop_counter < 10)
            //     {
            //         printf("in loop counter: %d\n", loop_counter);
            //         printf("\ntarget[i]->control_word (hex): ");
            //         for (size_t i = 0; i < NUM_TARGET; i++)
            //         {
            //             printf("%X ", target[i]->control_word);
            //         }
            //         printf("\nval[i]->status_word (hex): ");
            //         for (size_t i = 0; i < NUM_TARGET; i++)
            //         {
            //             printf("%X ", val[i]->status_word);
            //         }
            //         printf("\n");
            //     }
            // }




            if (should_print && loop_counter % 50 == 0)
            {
                // printf("\nvoltage: ");
                // for (int i = 0; i < NUM_TARGET; i++)
                // {
                //     READ<uint32>(i + 1, 0x6079, 0, buf32, "DC link circuit voltage", false);
                //     printf("%8d,", buf32);
                // }

                printf("\npos_tar:");
                for (int i = 0; i < NUM_TARGET; i++)
                {
                    printf("%8.5f,", (double)(target[i]->target_position) / gear_ratio(i) * (2 * M_PI) / 131072);
                }
                printf("\npos:    ");
                for (int i = 0; i < NUM_TARGET; i++)
                {
                    printf("%8.5f,", actual_position(i));
                }
                printf("\nprr:    "); // posotion error
                for (int i = 0; i < NUM_TARGET; i++)
                {
                    printf("%8.5f,", actual_position_error(i));
                }
                printf("\nvel:    ");
                for (int i = 0; i < NUM_TARGET; i++)
                {
                    printf("%8.5f,", actual_velocity(i));
                }
                printf("\ntor:    "); // torque in [mA]
                for (int i = 0; i < NUM_TARGET; i++)
                {
                    printf("%8.5f,", actual_torque(i)); // TODO compute torque conversion
                }
                printf("\n");

                // printf("\ntarget[i]->control_word (hex): ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%X ", target[i]->control_word);
                // }
                // printf("\n");

                // printf("val[i]->status_word (hex): ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%X ", val[i]->status_word);
                // }
                // printf("\n");

                // printf("target[i]->mode_of_operation: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", target[i]->mode_of_operation);
                // }
                // printf("\n");

                // printf("val[i]->mode_of_operation_disp: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", val[i]->mode_of_operation_disp);
                // }
                // printf("\n");

                // printf("target[i]->target_position: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", target[i]->target_position);
                // }
                // printf("\n");

                // printf("target[i]->target_velocity: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", target[i]->target_velocity);
                // }
                // printf("\n");

                // printf("target[i]->velocity_offset: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", target[i]->velocity_offset);
                // }
                // printf("\n");

                // printf("target[i]->target_torque: ");
                // for (size_t i = 0; i < NUM_TARGET; i++)
                // {
                //     printf("%d ", target[i]->target_torque);
                // }
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

private:
    int loop_counter = 0;
    char IOmap[4096];
    int expectedWKC;                    // Expected Working Counter value (used for data exchange validation).
    volatile int wkc;                   // Actual Working Counter value received
    boolean needlf = false;             // Flag to control newline printing.
    boolean inOP = FALSE;               // Flag indicating if slaves are in operational state.
    uint8 currentgroup = 0;             // Currently active EtherCAT group.
    boolean forceByteAlignment = FALSE; // Flag to control byte alignment of IO memory.

    // placeholder for reading/writing buffer
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    int8 sbuf8;
    int32 sbuf32;
};

#endif /* _ETHERCAT_MOTOR_H */
/*
 * File: assignment1.cpp
 *
 * Framework to implement Task 1 of the Advances in Computer Architecture lab
 * session. This uses the framework library to interface with tracefiles which
 * will drive the read/write requests
 *
 * Author(s): Michiel W. van Tol, Mike Lankamp, Jony Zhang,
 *            Konstantinos Bousias
 * Copyright (C) 2005-2017 by Computer Systems Architecture group,
 *                            University of Amsterdam
 *
 */

#include <systemc>
#include <iostream>

#include "psa.h"

using namespace std;
using namespace sc_core; // This pollutes namespace, better: only import what you need.

#define MEMORY_SIZE                  512
#define CACHE_LINE_SIZE_BYTES        32
#define CACHE_NUMBER_OF_LINES_IN_SET 8
#define CACHE_NUMBER_OF_SETS         128

// SC_MODULE(Memory)
// {

// public:
//     enum Function
//     {
//         FUNC_READ,
//         FUNC_WRITE
//     };

//     enum RetCode
//     {
//         RET_READ_DONE,
//         RET_WRITE_DONE,
//     };

//     sc_in<bool>     Port_CLK;
//     sc_in<Function> Port_Func;
//     sc_in<int>      Port_Addr;
//     sc_out<RetCode> Port_Done;
//     sc_inout_rv<32> Port_Data;

//     SC_CTOR(Memory)
//     {
//         SC_THREAD(execute);
//         sensitive << Port_CLK.pos();
//         dont_initialize();

//         m_data = new int[MEMORY_SIZE];
//     }

//     ~Memory()
//     {
//         delete[] m_data;
//     }

// private:
//     int* m_data;

//     void execute()
//     {
//         while (true)
//         {
//             wait(Port_Func.value_changed_event());

//             Function f = Port_Func.read();
//             int addr   = Port_Addr.read();
//             int data   = 0;
//             if (f == FUNC_WRITE)
//             {
//                 cout << sc_time_stamp() << ": MEM received write" << endl;
//                 data = Port_Data.read().to_int();
//             }
//             else
//             {
//                 cout << sc_time_stamp() << ": MEM received read" << endl;
//             }

//             // This simulates memory read/write delay
//             wait(99);

//             if (f == FUNC_READ)
//             {
//                 Port_Data.write( (addr < MEMORY_SIZE) ? m_data[addr] : 0 );
//                 Port_Done.write( RET_READ_DONE );
//                 wait();
//                 Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
//             }
//             else
//             {
//                 if (addr < MEMORY_SIZE)
//                 {
//                     m_data[addr] = data;
//                 }
//                 Port_Done.write( RET_WRITE_DONE );
//             }
//         }
//     }
// };

SC_MODULE(Cache)
{

public:
    enum Function
    {
        FUNC_READ,
        FUNC_WRITE
    };

    enum RetCode
    {
        RET_READ_DONE,
        RET_WRITE_DONE,
    };

    sc_in<bool>     Port_CLK;
    sc_in<Function> Port_Func;
    sc_in<int>      Port_Addr;
    sc_out<RetCode> Port_Done;
    sc_inout_rv<32> Port_Data;

    SC_CTOR(Cache)
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
        initialize_cache_arrays();
        cout << bit_mask_byte_in_line << ' ' << bit_mask_set_address << ' ' << bit_mask_tag << '\n';
    }

private:
    
    u_int8_t  **cache;
    u_int16_t **tags;

    u_int32_t bit_mask_byte_in_line = create_mask(0,  4);
    u_int32_t bit_mask_set_address  = create_mask(5, 15);
    u_int32_t bit_mask_tag          = create_mask(16, 31);

    void initialize_cache_arrays()
    {
        cache = new u_int8_t* [CACHE_NUMBER_OF_SETS];
        tags  = new u_int16_t*[CACHE_NUMBER_OF_SETS]; 

        for(int i = 0; i < CACHE_NUMBER_OF_SETS; i++)
        {
            cache[i] = new u_int8_t [CACHE_NUMBER_OF_LINES_IN_SET * CACHE_LINE_SIZE_BYTES];
            tags[i]  = new u_int16_t[CACHE_NUMBER_OF_LINES_IN_SET];
        }
    }

    u_int32_t create_mask(u_int8_t start_bit, u_int8_t end_bit)
    {
        u_int32_t mask = 0;
        for(u_int8_t i  = start_bit; i <= end_bit; i++)
        {
            mask |= 1 << i;
        }
        return mask;
    }

    void execute()
    {
        while (true)
        {
            wait(Port_Func.value_changed_event());

            Function f = Port_Func.read();
            int addr   = Port_Addr.read();

            u_int8_t byte_in_line = addr & bit_mask_byte_in_line;
            u_int16_t set_address = addr & bit_mask_set_address;
            u_int16_t tag         = addr & bit_mask_tag;

            int data   = 0;
            if (f == FUNC_WRITE)
            {
                cout << sc_time_stamp() << ": MEM received write" << endl;
                data = Port_Data.read().to_int();
            }
            else
            {
                cout << sc_time_stamp() << ": MEM received read" << endl;
            }

            u_int8_t set_index = set_address % CACHE_NUMBER_OF_SETS;

            cout << "Set index: " << set_index << '\n';

            if (f == FUNC_READ)
            {
                Port_Data.write( 0 );
                Port_Done.write( RET_READ_DONE );
                wait();
                Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
            }
            else
            {
                if (addr < MEMORY_SIZE)
                {
                    // m_data[addr] = data;
                }
                Port_Done.write( RET_WRITE_DONE );
            }
        }   
    }
};

SC_MODULE(CPU)
{

public:
    sc_in<bool>                Port_CLK;
    sc_in<Cache::RetCode>      Port_MemDone;
    sc_out<Cache::Function>    Port_MemFunc;
    sc_out<int>                Port_MemAddr;
    sc_inout_rv<32>            Port_MemData;

    SC_CTOR(CPU)
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
    }

private:
    void execute()
    {
        TraceFile::Entry    tr_data;
        Cache::Function  f;

        // Loop until end of tracefile
        while(!tracefile_ptr->eof())
        {
            // Get the next action for the processor in the trace
            if(!tracefile_ptr->next(0, tr_data))
            {
                cerr << "Error reading trace for CPU" << endl;
                break;
            }

            // To demonstrate the statistic functions, we generate a 50%
            // probability of a 'hit' or 'miss', and call the statistic
            // functions below
            // int j = rand() % 2;

            switch(tr_data.type)
            {
                case TraceFile::ENTRY_TYPE_READ:
                    f = Cache::FUNC_READ;
                    // if(j)
                    //     stats_readhit(0);
                    // else
                    //     stats_readmiss(0);
                    break;

                case TraceFile::ENTRY_TYPE_WRITE:
                    f = Cache::FUNC_WRITE;
                    // if(j)
                    //     stats_writehit(0);
                    // else
                    //     stats_writemiss(0);
                    break;

                case TraceFile::ENTRY_TYPE_NOP:
                    break;

                default:
                    cerr << "Error, got invalid data from Trace" << endl;
                    exit(0);
            }

            if(tr_data.type != TraceFile::ENTRY_TYPE_NOP)
            {
                Port_MemAddr.write(tr_data.addr);
                Port_MemFunc.write(f);

                if (f == Cache::FUNC_WRITE)
                {
                    cout << sc_time_stamp() << ": CPU sends write" << endl;

                    uint32_t data = rand();
                    Port_MemData.write(data);
                    wait();
                    Port_MemData.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                }
                else
                {
                    cout << sc_time_stamp() << ": CPU sends read" << endl;
                }

                wait(Port_MemDone.value_changed_event());

                if (f == Cache::FUNC_READ)
                {
                    cout << sc_time_stamp() << ": CPU reads: " << Port_MemData.read() << endl;
                }
            }
            else
            {
                cout << sc_time_stamp() << ": CPU executes NOP" << endl;
            }
            // Advance one cycle in simulated time
            wait();
        }

        // Finished the Tracefile, now stop the simulation
        sc_stop();
    }
};

int sc_main(int argc, char* argv[])
{
    try
    {
        // Get the tracefile argument and create Tracefile object
        // This function sets tracefile_ptr and num_cpus
        init_tracefile(&argc, &argv);

        // Initialize statistics counters
        stats_init();

        // Instantiate Modules
        Cache cache("L1 Cache");
        CPU   cpu("cpu");

        // Signals
        sc_buffer<Cache::Function> sigMemFunc;
        sc_buffer<Cache::RetCode>  sigMemDone;
        sc_signal<int>             sigMemAddr;
        sc_signal_rv<32>           sigMemData;

        // The clock that will drive the CPU and Cache
        sc_clock clk;

        // Connecting module ports with signals
        cache.Port_Func(sigMemFunc);
        cache.Port_Addr(sigMemAddr);
        cache.Port_Data(sigMemData);
        cache.Port_Done(sigMemDone);

        cpu.Port_MemFunc(sigMemFunc);
        cpu.Port_MemAddr(sigMemAddr);
        cpu.Port_MemData(sigMemData);
        cpu.Port_MemDone(sigMemDone);

        cache.Port_CLK(clk);
        cpu.Port_CLK(clk);

        cout << "Running (press CTRL+C to interrupt)... " << endl;


        // Start Simulation
        sc_start();

        // Print statistics after simulation finished
        stats_print();
    }

    catch (exception& e)
    {
        cerr << e.what() << endl;
    }

    return 0;
}

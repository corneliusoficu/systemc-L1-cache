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
#include <stdlib.h>

#include "psa.h"

using namespace std;
using namespace sc_core; // This pollutes namespace, better: only import what you need.

#define CACHE_LINE_SIZE_BYTES        32
#define CACHE_NUMBER_OF_LINES_IN_SET 8
#define CACHE_NUMBER_OF_SETS         128

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
    
    int **cache;
    int **tags;
    int **least_recently_updated;

    int bit_mask_byte_in_line = create_mask(0,  4);
    int bit_mask_set_address  = create_mask(5, 11);
    int bit_mask_tag          = create_mask(12, 31);

    void initialize_cache_arrays()
    {
        cache                   = new int*[CACHE_NUMBER_OF_SETS];
        tags                    = new int*[CACHE_NUMBER_OF_SETS]; 
        least_recently_updated  = new int*[CACHE_NUMBER_OF_SETS];

        for(int i = 0; i < CACHE_NUMBER_OF_SETS; i++)
        {
            least_recently_updated[i] = new int[CACHE_NUMBER_OF_LINES_IN_SET];
            cache[i]                  = new int[CACHE_NUMBER_OF_LINES_IN_SET * CACHE_LINE_SIZE_BYTES];
            tags[i]                   = new int[CACHE_NUMBER_OF_LINES_IN_SET];

            for(int j = 0; j < CACHE_NUMBER_OF_LINES_IN_SET; j++)
            {
                tags[i][j] = -1;
            }
        }
    }

    int create_mask(int start_bit, int end_bit)
    {
        int mask = 0;
        for(int i  = start_bit; i <= end_bit; i++)
        {
            mask |= 1 << i;
        }
        return mask;
    }

    int get_index_of_line_in_set(int set_index, int tag)
    {
        for(int index = 0; index < CACHE_NUMBER_OF_LINES_IN_SET; index++)
        {
            if(tags[set_index][index] == tag)
            {
                return index;
            }
        }
        return -1;
    }

    void update_lru(int set_address, int line_in_set_index)
    {
        for(int i = 0; i < CACHE_NUMBER_OF_SETS; i++)
        {
            if(least_recently_updated[set_address][i] < least_recently_updated[set_address][line_in_set_index])
            {
                least_recently_updated[set_address][i]++;
            }
        }
        least_recently_updated[set_address][line_in_set_index] = 0;
    }

    int get_lru_line(int set_address){ //returns index of the lru line
        int max = 0;
        int max_index = 0;

        for(int i = 0; i < CACHE_NUMBER_OF_LINES_IN_SET; i++)
        {
            if(least_recently_updated[set_address][i] > max)
            {
                max =  least_recently_updated[set_address][i];
                max_index = i;
            }

        }
        return max_index;
    }

    void handle_cache_read(int set_address, int tag, int byte_in_line)
    {
        int line_in_set_index = get_index_of_line_in_set(set_address, tag);
                
        if(line_in_set_index == -1)
        {
            stats_readmiss(0);
            //simulate read from memory and store in cache
            wait(99);
            line_in_set_index = get_lru_line(set_address);
            tags[set_address][line_in_set_index] = tag;
            cache[set_address][line_in_set_index * CACHE_LINE_SIZE_BYTES + byte_in_line] = rand() % 1000 + 1;
        }
        else
        {
            update_lru(set_address, line_in_set_index);
            stats_readhit(0);
        }
        
        update_lru(set_address, line_in_set_index);
        
        Port_Data.write(cache[set_address][line_in_set_index * CACHE_LINE_SIZE_BYTES + byte_in_line]);
        Port_Done.write(RET_READ_DONE);
        
        wait();
        Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
    }

    void handle_cache_write(int addr, int set_address, int tag, int byte_in_line, int data)
    {
        int line_in_set_index = get_index_of_line_in_set(set_address, tag);

        if(line_in_set_index == -1)
        {
            stats_writemiss(0);
            line_in_set_index = get_lru_line(set_address);
            tags[set_address][line_in_set_index] = tag;
            wait(99);
        }
        else
        {
            stats_writehit(0);
            update_lru(set_address, line_in_set_index);
            wait();
        }

        cache[set_address][line_in_set_index * CACHE_NUMBER_OF_LINES_IN_SET] = data;
        Port_Done.write(RET_WRITE_DONE);
    }

    void execute()
    {
        while (true)
        {
            wait(Port_Func.value_changed_event());

            Function f = Port_Func.read();
            int addr   = Port_Addr.read();

            int byte_in_line = addr & bit_mask_byte_in_line;       // Obtaining value for bits 0 - 4, no shifting required
            int set_address  = (addr & bit_mask_set_address) >> 5; // Shifting to right to obtain value for bits 5  - 11
            int tag          = (addr & bit_mask_tag) >> 12;        // Shifting to right to obtain value for bits 12 - 31

            if (f == FUNC_WRITE)
            {
                cout << sc_time_stamp() << ": MEM received write" << endl;
                int data = Port_Data.read().to_int();
                handle_cache_write(addr, set_address, tag, byte_in_line, data);
            }
            else
            {
                cout << sc_time_stamp() << ": MEM received read" << endl;
                handle_cache_read(set_address, tag, byte_in_line);
            }
        }   
    }
};

SC_MODULE(CPU)
{

public:
    sc_in<bool>             Port_CLK;
    sc_in<Cache::RetCode>   Port_CacheDone;
    sc_out<Cache::Function> Port_CacheFunc;
    sc_out<int>             Port_CacheAddr;
    sc_inout_rv<32>         Port_CacheData;

    SC_CTOR(CPU)
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
    }

private:
    void execute()
    {
        TraceFile::Entry tr_data;
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

            switch(tr_data.type)
            {
                case TraceFile::ENTRY_TYPE_READ:
                    f = Cache::FUNC_READ;
                    break;

                case TraceFile::ENTRY_TYPE_WRITE:
                    f = Cache::FUNC_WRITE;
                    break;

                case TraceFile::ENTRY_TYPE_NOP:
                    break;

                default:
                    cerr << "Error, got invalid data from Trace" << endl;
                    exit(0);
            }

            if(tr_data.type != TraceFile::ENTRY_TYPE_NOP)
            {
                Port_CacheAddr.write(tr_data.addr);
                Port_CacheFunc.write(f);

                if (f == Cache::FUNC_WRITE)
                {
                    cout << sc_time_stamp() << ": CPU sends write" << endl;

                    uint32_t data = rand();
                    Port_CacheData.write(data);
                    wait();
                    Port_CacheData.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                }
                else
                {
                    cout << sc_time_stamp() << ": CPU sends read" << endl;
                }

                wait(Port_CacheDone.value_changed_event());

                if (f == Cache::FUNC_READ)
                {
                    cout << sc_time_stamp() << ": CPU reads: " << Port_CacheData.read() << endl;
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
        sc_buffer<Cache::Function> sigCacheFunc;
        sc_buffer<Cache::RetCode>  sigCacheDone;
        sc_signal<int>             sigCacheAddr;
        sc_signal_rv<32>           sigCacheData;

        // The clock that will drive the CPU and Cache
        sc_clock clk;

        // Connecting module ports with signals
        cache.Port_Func(sigCacheFunc);
        cache.Port_Addr(sigCacheAddr);
        cache.Port_Data(sigCacheData);
        cache.Port_Done(sigCacheDone);

        cpu.Port_CacheFunc(sigCacheFunc);
        cpu.Port_CacheAddr(sigCacheAddr);
        cpu.Port_CacheData(sigCacheData);
        cpu.Port_CacheDone(sigCacheDone);

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

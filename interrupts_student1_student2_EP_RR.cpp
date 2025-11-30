/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @author Ajan Balaganesh
 * @author Kyle Deng
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include "interrupts_student1_student2.hpp"
#include<map>

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}


std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;



    
    std::map<int,unsigned int>    time_to_io;
    std::map<int,unsigned int>    io_remaining;
    unsigned int                  quantum  = 100;
    unsigned int                  qcounter = 0;

    for (auto &p : list_processes) {
        time_to_io[p.PID]   = p.io_freq;
        io_remaining[p.PID] = 0;
    }    
    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for(auto &process : list_processes) {
            if(process.arrival_time == current_time) {//check if the AT = current time
                //if so, assign memory and put the process into the ready queue
                assign_memory(process);

                process.state = READY;  //Set the process state to READY
                ready_queue.push_back(process); //Add the process to the ready queue
                job_list.push_back(process); //Add it to the list of processes

                execution_status += print_exec_status(current_time, process.PID, NEW, READY);
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the ready queue


        for (std::size_t i = 0; i < wait_queue.size(); ) {
            PCB p = wait_queue[i];

            if (io_remaining[p.PID] > 0) {
                io_remaining[p.PID]--;
            }
   //put process in ready queue once io done
            if (io_remaining[p.PID] == 0) {
                states old = p.state;
                p.state = READY;
                sync_queue(job_list, p);
                ready_queue.push_back(p);
                execution_status += print_exec_status(current_time, p.PID, old, READY);
                wait_queue.erase(wait_queue.begin() + i);
            } else {
                ++i;
            }
        }
        /////////////////////////////////////////////////////////////////
        if (running.state == RUNNING) {

            if (running.remaining_time > 0) {
                running.remaining_time--;
            }

            if (running.io_freq > 0 && time_to_io[running.PID] > 0) {
                time_to_io[running.PID]--;
            }

            qcounter++;

       // Process done
            if (running.remaining_time == 0) {
                states old = RUNNING;
                terminate_process(running, job_list);
                execution_status += print_exec_status(current_time, running.PID, old, TERMINATED);
                idle_CPU(running);
                qcounter = 0;
            }
      // IO gets trigerred and process gets put in wait queue
            else if (running.io_freq > 0 && time_to_io[running.PID] == 0) {
                states old = RUNNING;
                running.state = WAITING;
                sync_queue(job_list, running);
                io_remaining[running.PID] = running.io_duration;
                time_to_io[running.PID]   = running.io_freq;
                execution_status += print_exec_status(current_time, running.PID, old, WAITING);
                wait_queue.push_back(running);
                idle_CPU(running);
                qcounter = 0;
            }
            else {
                sync_queue(job_list, running);
            }
        }
        //////////////////////////SCHEDULER//////////////////////////////
        
       //using index this time because we kept running into memory error

        if (!ready_queue.empty()) {

            int best_idx = 0;

        // schedule up the highest priority process in ready queue for next
            for (int i = 1; i < ready_queue.size(); ++i) {
                if (ready_queue[i].PID < ready_queue[best_idx].PID) {
                    best_idx = i;
                }
            }

            PCB next = ready_queue[best_idx];
           
             // switch only if higher priority process or quantum's done or state is unassigned
            bool should_switch = (running.state == NOT_ASSIGNED) || (next.PID < running.PID) ||(qcounter == quantum);

            if (should_switch) {
                if (running.state == RUNNING) {
                    states old = RUNNING;
                    running.state = READY;
                    sync_queue(job_list, running);
                    ready_queue.push_back(running);
                    execution_status += print_exec_status(current_time, running.PID, old, READY);
                }

                ready_queue.erase(ready_queue.begin() + best_idx);

                states old = next.state;
                next.state = RUNNING;
                if (next.start_time == -1) next.start_time = current_time;
                running = next;
                sync_queue(job_list, running);
                execution_status += print_exec_status(current_time, running.PID, old, RUNNING);
                qcounter = 0;
            }
        }

        current_time++;
        /////////////////////////////////////////////////////////////////

    }
    
    //Close the output table
    execution_status += print_exec_footer();

    return std::make_tuple(execution_status);
}


int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}

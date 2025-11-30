/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @author Ajan Balaganesh
 * @author Kyle Deng
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include "interrupts_student1_student2.hpp"
#include <map>

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

//helps keeps state
std::map<int,int> pmap;
std::map<int,unsigned int> time_to_io;
std::map<int,unsigned int> io_remaining;
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

    pmap.clear();
    time_to_io.clear();
    io_remaining.clear();

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
        for (auto it = wait_queue.begin(); it != wait_queue.end();) {
            PCB &p = *it;

            if (io_remaining[p.PID] > 0) {
                io_remaining[p.PID]--;
            }
	
            // put the process back in the ready queue when io finishes
            if (io_remaining[p.PID] == 0) {
                states old_state = p.state;
                p.state = READY;
                sync_queue(job_list, p);
                ready_queue.push_back(p);
                execution_status += print_exec_status(current_time, p.PID, old_state, READY);
                it = wait_queue.erase(it);
            } else {
                ++it;
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

        //process has no more run time
            if (running.remaining_time == 0) {
                states old_state = RUNNING;
                terminate_process(running, job_list);
                execution_status += print_exec_status(current_time, running.PID, old_state, TERMINATED);
                idle_CPU(running);
            }
       //io is triggered and process gets put in waiting
            else if (running.io_freq > 0 && time_to_io[running.PID] == 0) {
                states old_state = RUNNING;
                running.state = WAITING;
                sync_queue(job_list, running);

                io_remaining[running.PID] = running.io_duration;
                time_to_io[running.PID]   = running.io_freq;

                execution_status += print_exec_status(current_time, running.PID, old_state, WAITING);

                wait_queue.push_back(running);
                idle_CPU(running);
            }
            else {
                sync_queue(job_list, running);
            }
        }
        //////////////////////////SCHEDULER//////////////////////////////
        if (running.state == NOT_ASSIGNED && !ready_queue.empty()) {
            auto best_it = ready_queue.begin();
          //Find highest priority process to schedule next
            for (auto it = ready_queue.begin(); it != ready_queue.end(); ++it) {
                if (it->PID < best_it->PID) {
                    best_it = it;
                }
            }
         // scheduling the next process
            PCB next = *best_it;
            ready_queue.erase(best_it);

            states old_state = next.state;
            next.state = RUNNING;
            if (next.start_time == -1) {
                next.start_time = current_time;
            }

            running = next;
            sync_queue(job_list, running);
            execution_status += print_exec_status(current_time, running.PID, old_state, RUNNING);
        }
        /////////////////////////////////////////////////////////////////
	current_time++;
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

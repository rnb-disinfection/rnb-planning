//
// Created by rnb on 21. 2. 6..
//

#ifndef LATTICIZER_TIMER_H
#define LATTICIZER_TIMER_H
#include <chrono>


/**
 * @class TIMER
 * @brief simple timer
 */
class Timer{
public:
    std::chrono::system_clock::time_point t0;
    double timescale;
    /**
     * @param timescale scale of timeing. by default =1000 (ms)
     */
    Timer(double timescale=1000): timescale(timescale){}

    /**
     * @brief mark starting time
     */
    void tic(){
        t0 = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief return past time from last call of tic()
     */
    double toc(){
        std::chrono::system_clock::time_point tt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = tt - t0;
        return elapsed.count()*timescale;
    }
};

#endif //LATTICIZER_TIMER_H

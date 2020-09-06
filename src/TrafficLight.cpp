#include <iostream>
#include <random>
#include <future>
#include <mutex>
#include <algorithm>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */
template <typename T>
T MessageQueue<T>::receive()
{
    std::unique_lock<std::mutex> uLock(_mutex);
    
    _condition.wait(uLock, [this]{ return(!_queue.empty());});
    
    // remove last vector element from queue
    T msg = std::move(_queue.back());
    _queue.pop_back();

    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    
    //Clear Queue before pushing the msg.
    _queue.clear();
    _queue.push_back(std::move(msg));
    
    _condition.notify_one();
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    while(true) {
        if (_msgqueue.receive() == TrafficLightPhase::green) {
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    //Add a protection there because the Thread function of TrafficLight can modify the currentPhase when we try to access it from Intersection
    std::lock_guard<std::mutex> lock(_mutex);
    
    return _currentPhase;
}

void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    //Init first cycleDuration
    std::srand(std::time(0));
    double cycleDuration = std::rand() % 2000 + 4000;

    // init stop watch
    lastUpdate = std::chrono::system_clock::now();
    
    while(true) {
        
        //Wait 1 ms between two cycles.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        
        if (timeSinceLastUpdate >= cycleDuration) {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_currentPhase == TrafficLightPhase::green) {
                _currentPhase = TrafficLightPhase::red;
            }
            else
            {
                _currentPhase = TrafficLightPhase::green;
            }
            
            _msgqueue.send(std::move(TrafficLightPhase(_currentPhase)));
            
            lastUpdate = std::chrono::system_clock::now();
            cycleDuration = std::rand() % 2000 + 4000;
        }
    }
}

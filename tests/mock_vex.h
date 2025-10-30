#ifndef __MOCK_VEX_H_INCLUDED__
#define __MOCK_VEX_H_INCLUDED__

#include <thread>
#include <mutex>
#include <chrono>

namespace vex
{
    enum rotationUnits
    {
        rev
    };

    class rotation
    {
    private:
        double mockRevs;

    public:
        rotation(double revs) : mockRevs(revs) {};

        double position(rotationUnits rotUnits) { return mockRevs; }

        void setMockPosition(double revs)
        {
            mockRevs = revs;
        }
    };

    // Mock VEX threading (errors can be ignored)
    using thread = std::thread;
    using mutex = std::mutex;

    namespace this_thread {
        inline void sleep_for(int milliseconds) {
            std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
        }
    }

}

#endif
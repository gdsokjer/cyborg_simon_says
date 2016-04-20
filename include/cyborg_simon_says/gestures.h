#ifndef CYBORG_SIMON_SAYS_GESTURES_H
#define CYBORG_SIMON_SAYS_GESTURES_H

#include "cyborg_simon_says/event.h"

#include <k2_client/k2_client.h>
#include <k2_client/BodyArray.h>

namespace cyborg
{
    namespace simon_says
    {
        enum class Gesture : int
        {
            wave          = 0,
            stop          = 1,
            liftRightFoot = 2,
            standOnToes   = 3,
            flexnes       = 4
        };

        class GestureEvent : public Event
        {
            public:
                GestureEvent(Gesture);
                virtual std::string announcement();
                virtual bool        completed();
                virtual void        start();
                virtual double      timeout();
            private:
                Gesture gesture_;
        };

        namespace gestures
        {
            void bodyArrayMessageHandler(const k2_client::BodyArray);
        }
    }
}

#endif // CYBORG_SIMON_SAYS_GESTURES_H


#include "ros/ros.h"
#include "cyborg_simon_says/event.h"
#include "cyborg_simon_says/game.h"
#include "cyborg_simon_says/ros_output.h"

#include <memory>
#include <string>
#include <vector>

class ButtonPressEvent : public cyborg::simon_says::Event
{
    public:
        ButtonPressEvent(const char key) : key_(key), completed_(false) {}

        virtual std::string announcement()
        {
            return std::string("Please press the ") + key_;
        }

        virtual bool completed()
        {
            return completed_;
        }

        void set(const char key)
        {
            if (key == key_)
            {
                completed_ = true;
            }
        }

        virtual void start()
        {
            completed_ = false;
        }

        virtual double timeout()
        {
            return 5.0;
        }

    private:
        char key_;
        bool completed_;
};

int main(int argc, char** argv)
{
    set_conio_terminal_mode();

    ros::init(argc, argv, "cyborg_simon_says");
    ros::NodeHandle node;

    auto startEvent = std::make_shared<ButtonPressEvent>('s');

    std::vector<std::shared_ptr<cyborg::simon_says::Event>> eventLibrary
    {
        std::make_shared<ButtonPressEvent>('a'),
        std::make_shared<ButtonPressEvent>('b'),
        std::make_shared<ButtonPressEvent>('c'),
        std::make_shared<ButtonPressEvent>('d'),
        std::make_shared<ButtonPressEvent>('e')
    };

    ros::Publisher p;
    auto output = std::make_shared<cyborg::simon_says::RosOutput>(p);

    cyborg::simon_says::Game game(
        startEvent, eventLibrary, output);

    game.setStringValue(cyborg::simon_says::Game::String::gameStart, "GAME STARTED");
    game.setStringValue(cyborg::simon_says::Game::String::gameFailed, "GAME FAILED");
    game.setStringValue(cyborg::simon_says::Game::String::gameCompleted, "GAME COMPLETED");
    game.setStringValue(cyborg::simon_says::Game::String::eventCompleted, "EVENT COMPLETED");
    game.setStringValue(cyborg::simon_says::Game::String::gameTimeout, "GAME TIMEDOUT");

    game.setNumberValue(cyborg::simon_says::Game::Number::beginEventDelay, 2.0);
    game.setNumberValue(cyborg::simon_says::Game::Number::gameRestartDelay, 5.0);
    game.setNumberValue(cyborg::simon_says::Game::Number::challengeCount, 3.0);

    ROS_INFO_NAMED("cyborg_simon_says", "cyborg_simon_says ready to serve!");

    ros::Rate loopRate(2);

    while (true)
    {
        ros::spinOnce();
        game.process();

        // Need to be fixed; only for testing anyway.
        const auto key = getch();

        if (key >= 0 && key != 10)
        {
            startEvent->set(key);

            for (const auto& e : eventLibrary)
            {
                dynamic_cast<ButtonPressEvent&>(*e).set(key);
            }
        }
    }
}


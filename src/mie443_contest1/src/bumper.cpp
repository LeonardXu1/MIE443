#include "../include/bumper.h"
bool bumperBehaviourComplete = false;
bool bumperRandomComplete = false;
int bumperRandomNum;
int randomAngleDegrees;
double randomAngleRadians;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
std::string bumperPosition;
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    bumper[msg->bumper] = msg->state;
}

std::string bumperPressedPosition()
{
    std::string position;
    if (bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "LEFT";
        return position;
    }
    if (bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "RIGHT";
        return position;
    }
    if (bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "CENTER";
        return position;
    }
    return "NONE";
}

bool isBumperPressed()
{
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        if (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED)
        {
            return true;
        }
    }
    return false;
}
std::string savedBumperPosition()
{
    if (isBumperPressed() == true)
    {
        bumperPosition = bumperPressedPosition();
    }

    return bumperPosition;
}

void bumperBehaviour()
{
    bool taskComplete;
    int step = getStep();
    if (step == 0)
    {
        savedBumperPosition();
        savePos();
        takeStep();
    }
    else if (step == 1)
    {
        taskComplete = moveDistance(0.1, SLOW_LINEAR, BACKWARD);

        if (taskComplete)
        {
            takeStep();
        }
    }
    else if (step == 2)
    {
        savePos();
        takeStep();
    }
    else if (step == 3)
    {

        if (savedBumperPosition() == "LEFT")
        {
            // ROS_INFO("left bumper got pressed");

            taskComplete = moveAngle(50 * M_PI / 180, SLOW_ANGULAR, CW);

            if (taskComplete)
            {
                bumperBehaviourComplete = true;
                takeStep();
            }
        }

        else if (savedBumperPosition() == "RIGHT")
        {
            // ROS_INFO("right bumper got pressed");

            taskComplete = moveAngle(50 * M_PI / 180, SLOW_ANGULAR, CCW);

            if (taskComplete)
            {
                bumperBehaviourComplete = true;
                takeStep();
            }
        }
        else if (savedBumperPosition() == "CENTER")
        {
            // ROS_INFO("center bumper got pressed");

            if (bumperRandomComplete == false)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<int> dis(-1, 1);
                bumperRandomNum = dis(gen);
                while (bumperRandomNum == 0)
                {
                    // randomNum=dis(gen);
                    if (bumperRandomNum != 0)
                    {
                        break;
                    }
                    else
                    {
                        bumperRandomNum = dis(gen);
                    }
                }
                std::uniform_int_distribution<int> angleDis(95, 160);
                randomAngleDegrees = angleDis(gen);
                randomAngleRadians = randomAngleDegrees * M_PI / 180.0;
            }
            // ROS_INFO("Random angle: %d degrees, %f radians", randomAngleDegrees, randomAngleRadians);
            // ROS_INFO("Random direction %i", bumperRandomNum);
            bumperRandomComplete = true;
            taskComplete = moveAngle(randomAngleRadians, SLOW_ANGULAR, bumperRandomNum);

            if (taskComplete)
            {
                takeStep();
                bumperRandomComplete = false;
                bumperBehaviourComplete = true;
            }
        }
    }
    else {
        resetState();
    }
}
void resetBumperCompletion()
{
    bumperBehaviourComplete = false;
}
bool isBumperBehaviourComplete()
{
    return bumperBehaviourComplete;
}
#include "StdAfx.h"
#include <Windows.h>
#include "AI_Input_Example.h"

int main()
{
    using namespace Module::Input;
    using namespace Framework::Base;
    AI_Example tester;
    //TODO setup hooks here
    Goal& goal = tester.GetGoal();
    //run the goal
    goal.Activate();
    while (goal.GetStatus() == Goal::eActive)
    {
        goal.Process(0.010);
        Sleep(10);
    }
}

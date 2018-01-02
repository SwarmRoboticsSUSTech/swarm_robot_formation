#include "formation/discussion/desirepositioninfo.h"

using namespace formation;

DesirePositionInfo::DesirePositionInfo()
{

}

DesirePositionInfo::DesirePositionInfo(int position, int time)
{
  this->Position = position;
  this->Time = time;
}

void DesirePositionInfo::setPosition(int position)
{
  this->Position = position;
}

int DesirePositionInfo::getPosition()
{
  return this->Position;
}

void DesirePositionInfo::setTime(int time)
{
  this->Time = time;
}

int DesirePositionInfo::getTime()
{
  return this->Time;
}

#include "desirepositioninfo.h"

desirepositioninfo::desirepositioninfo()
{

}

desirepositioninfo::desirepositioninfo(int position, int time)
{
  this->Position = position;
  this->Time = time;
}

void desirepositioninfo::setPosition(int position)
{
  this->Position = position;
}

int desirepositioninfo::getPosition()
{
  return this->Position;
}

void desirepositioninfo::setTime(int time)
{
  this->Time = time;
}

int desirepositioninfo::getTime()
{
  return this->Time;
}

#ifndef DESIREPOSITIONINFO_H
#define DESIREPOSITIONINFO_H

namespace formation {

class DesirePositionInfo
{
public:
  DesirePositionInfo();
  DesirePositionInfo(int position, int time);
  void setPosition(int position);
  int getPosition();
  void setTime(int time);
  int getTime();

private:
  int Position;
  int Time;
};

} /* namespace formation */

#endif // DESIREPOSITIONINFO_H

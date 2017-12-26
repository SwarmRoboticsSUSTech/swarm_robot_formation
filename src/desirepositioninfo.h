#ifndef DESIREPOSITIONINFO_H
#define DESIREPOSITIONINFO_H


class desirepositioninfo
{
public:
  desirepositioninfo();
  desirepositioninfo(int position, int time);
  void setPosition(int position);
  int getPosition();
  void setTime(int time);
  int getTime();

private:
  int Position;
  int Time;
};

#endif // DESIREPOSITIONINFO_H

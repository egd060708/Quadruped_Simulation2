#pragma once
#include <iostream>
#include <math.h>

class multiCircle {
private:
	int count = 0;
	double period_up = 0;
	double period_down = 0;
	double last_real = 0;
public:
	multiCircle(double _period_up, double _period_down)
	{
		this->period_up = _period_up;
		this->period_down = _period_down;
	}
	multiCircle(double _period)
	{
		this->period_up = abs(_period);
		this->period_down = -abs(_period);
	}
	double f(double _real)
	{
		if ((_real - last_real) < (period_up - period_down) * -0.5)
		{
			count++;// 圈数上溢
		}
		else if ((_real - last_real) > (period_up - period_down) * 0.5)
		{
			count--;// 圈数下溢
		}
		last_real = _real;
		return (_real + (period_up - period_down) * count);
	}
	void restart(double _real)
	{
		last_real = _real;
		count = 0;
	}
};
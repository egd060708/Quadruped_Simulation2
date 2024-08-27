/*! @file	dataDisplay.h
 *  @brief	webots��ͼ������
 *	@author	zzr
 *  @date	2023.9.21
 *
 *	����API���webots�ٷ��ĵ�
 */
#pragma once
#include "myMatrices.h"
#include <webots/Display.hpp>
#include "Upper_Public.h"
using namespace webots;
USING_NAMESPACE_MM

#define DATAMAX 10.f	// �������
#define ANXISDIVID 3	// �����Ữ�ָ���
#define ANXISCOLOR getcolor(100,100,100)	// ��������ɫ

/* ����ģ���ࣺ�Ӵ����ȣ��߶� */
template<uint8_t channelNum>
class dataDisplay {
private:
	myMatrices<int>* anxis;			// ����ϵ����
	myMatrices<int>* currentDisp;	// ��ǰҪ���ľ���
	myMatrices<int>* lastDisp;		// �ϴλ��ľ���
	myMatrices<uint8_t>* isUpdate;	// �Ƿ���Ҫ���»�ͼ
	myMatrices<int>* transfer;		// ת�ƾ���
	Display* tag;

	uint16_t width = 0;
	uint16_t height = 0;

	/* ���ö�Ӧͨ����ɫ */
	int getcolor(int r, int g, int b)
	{
		return (r << 16 | g << 8 | b);
	}

	/* ��ֵ�������꣨������ת�� */
	int num2row(double _num)
	{
		double temp = _num / DATAMAX * height / 2.f;// ��������ݵ����������λ��
		int std_temp = (int)temp;
		if (temp - std_temp > 0.5)
		{
			std_temp += 1;// ���������������
		}
		int fact_temp = height / 2 - std_temp - 1;//��Ӧ���ص��λ��
		return upper::constrain(fact_temp, 0, height - 1);
	}

	/* ���»������� */
	void currentUpdate(double _data[channelNum])
	{
		*currentDisp = (*currentDisp) * (*transfer);// ��������һ��
		for (int i = 0; i < channelNum; i++)
		{
			int _color;
			switch (i)
			{
			case 0: _color = getcolor(255, 0, 0); break;// ��
			case 1: _color = getcolor(0, 255, 0); break;// ��
			case 2: _color = getcolor(0, 0, 255); break;// ��
			case 3: _color = getcolor(0, 255, 255); break;// ��
			case 4: _color = getcolor(255, 0, 255); break;// ��
			case 5: _color = getcolor(255, 255, 0); break;// ��
			default:break;
			}
			currentDisp->setElement(num2row(_data[i]), width - 1, _color);// �Ѹ��µ���ɫд�����һ��������
		}

		// ������ϵ������������
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int dataPixel = currentDisp->getElement(i, j);
				int anxisPixel = anxis->getElement(i, j);
				if (dataPixel == 0 && anxisPixel == ANXISCOLOR)
				{
					currentDisp->setElement(i, j, ANXISCOLOR);
				}
			}
		}
	}

	/* �ж�Ҫ���»��Ƶĵ� */
	void drawingJudge()
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (currentDisp->getElement(i, j) != lastDisp->getElement(i, j))
				{
					isUpdate->setElement(i, j, 1);// ����Ҫ���ĵ����ص㸳ֵ1
				}
				else
				{
					isUpdate->setElement(i, j, 0);
				}
			}
		}
	}

public:
	/* ���캯�� */
	dataDisplay(Display* _tag):tag(_tag){
		static_assert((channelNum > 0) && (channelNum <= 6),
			"display channel number should be in [1,6]");// ����ͨ�������Ƿ��ڷ�Χ��

		width = tag->getWidth();	// ��ȡ�������
		height = tag->getHeight();	// ��ȡ����߶�

		currentDisp = new myMatrices<int>(height, width);
		lastDisp = new myMatrices<int>(height, width);
		isUpdate = new myMatrices<uint8_t>(height, width);
		transfer = new myMatrices<int>(width);
		anxis = new myMatrices<int>(height, width);

		for (int i = 0; i < width -1; i++)
		{
			transfer->setElement(i + 1, i,1); // ����Ӧλ�ø�1
		}

		// ��������ϵ����
		int anxis_color = ANXISCOLOR;
		for (int i = 0; i < ANXISDIVID; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				anxis->setElement(num2row(i * DATAMAX / (double)ANXISDIVID), j, anxis_color);
				anxis->setElement(num2row(-i * DATAMAX / (double)ANXISDIVID), j, anxis_color);
			}
		}
	}

	/* ���߻��� */
	void drawPixel()
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (isUpdate->getElement(i, j) == 1)
				{
					tag->setColor(currentDisp->getElement(i,j));// ȡ����Ӧ��ɫ
					tag->drawPixel(j, i);// ����
				}
			}
		}
	}

	/* ���ú��� */
	void sendCtrl(double _data[channelNum])
	{
		currentUpdate(_data);
		drawingJudge();
		drawPixel();
		*lastDisp = *currentDisp;// ���浱ǰ����״̬
	}

};
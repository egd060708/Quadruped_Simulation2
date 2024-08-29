#pragma once
#include <iostream>
#include <stdio.h>
#include <string>
#include <Windows.h>

using namespace std;

typedef struct _vofaFrame {
	float fdata[21];
	unsigned char tail[4]{ 0x00, 0x00, 0x80, 0x7f };
}vofaFrame;

class VOFA {
private:
	vofaFrame frame;
	FILE* ffmpeg;
	string process_path;
public:
	VOFA(string _process_path) {
		process_path = _process_path;
		string ffmpeg_command = process_path + string(" -addr localhost:") + string("7299");
		// 打开子进程
		ffmpeg = _popen(process_path.c_str(), "w");
	}
	~VOFA()
	{
		// 关闭子进程
		_pclose(ffmpeg);
	}
	void dataTransmit(float _data[9], int _sleepT)
	{
		memcpy(frame.fdata, _data, sizeof(frame.fdata));
		/* 调用fwrite向子程序写字符串 */
		fwrite(&frame, 1, sizeof(frame), ffmpeg);
		/* 刷新缓存，必须做*/
		fflush(ffmpeg);
		// 系统休眠
		Sleep(abs(_sleepT));
	}
};
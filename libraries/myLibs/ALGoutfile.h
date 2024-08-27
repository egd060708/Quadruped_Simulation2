/*
 * @Author: lu 2231625449@qq.com
 * @Date: 2024-04-29 01:05:51
 * @LastEditors: lu 2231625449@qq.com
 * @LastEditTime: 2024-05-10 16:43:55
 * @FilePath: /webots_autodriving_drone/traffic_project_bridge/libraries/myLibs/ALGoutfile.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

// void Exel_Output(string name, std::ios_base::openmode my_mode, float a[], int j, string discription); //向文件写入数组数据
// void file_create(string name, std::ios_base::openmode my_mode);//建立文件

//向文件写入数组数据
//参数1：文件名称，参数2：文件输出模式，参数3：数组，参数4：数组长度，参数5：表头描述
void Exel_Output(string name,std::ios_base::openmode my_mode,float a[],int j,string discription)
{
	ofstream outfile(name, my_mode);
	if(!outfile)
	{
		cerr<<"open error!"<<endl;
		exit(1);
	}
	// cout << name << " ok" << endl;
    outfile << discription << "\t"; 	//建立表头描述
    for (int i = 0; i < j;i++)
	{
		outfile<< a[i] <<"\t";			//把数组所有数据写在这一行
	}
    outfile << "\n";					//指针指到下一行
    outfile.close();
}

void Txt_Output(string name,std::ios_base::openmode my_mode,double a[],int j){
    ofstream outfile(name, my_mode);
	if(!outfile)
	{
		cerr<<"open error!"<<endl;
		exit(1);
	}
	// cout << name << " ok" << endl;
    for (int i = 0; i < j;i++)
	{
        if(i==j-1){
            outfile<< a[i];
        }
        else{
            outfile<< a[i] <<",";			//把数组所有数据写在这一行
        }
	}
    outfile << "\n";					//指针指到下一行
    outfile.close();
}

void Txt_Output(string name,std::ios_base::openmode my_mode,double a[], int j,string extraString){
    ofstream outfile(name, my_mode);
	if(!outfile)
	{
		cerr<<"open error!"<<endl;
		exit(1);
	}
	// cout << name << " ok" << endl;
    for (int i = 0; i < j;i++)
	{
		outfile << a[i] <<",";
	}
	outfile << extraString;
    outfile << "\n";					//指针指到下一行
    outfile.close();
}

void Txt_Output(string name,std::ios_base::openmode my_mode,float *a,int j){
    ofstream outfile(name, my_mode);
	if(!outfile)
	{
		cerr<<"open error!"<<endl;
		exit(1);
	}
	// cout << name << " ok" << endl;
    for (int i = 0; i < j;i++)
	{
        if(i==j-1){
            outfile<< a[i];
        }
        else{
            outfile<< a[i] <<",";			//把数组所有数据写在这一行
        }
	}
    outfile << "\n";					//指针指到下一行
    outfile.close();
}

string Txt_Input(string name){
	ifstream inputFile(name);
    // 检查文件是否成功打开
    if (!inputFile) {
        std::cerr << "Unable to open file example.txt" << std::endl;
        return string("error");
    }
    string line;
	string out;
    // 逐行读取文件内容并输出到控制台
    while (getline(inputFile, line)) {
		out += line;
    }
    // 关闭文件
    inputFile.close();
	
	return out;
}

void Txt_Input(string name, string *buffer, int size){
	ifstream inputFile(name);
    // 检查文件是否成功打开
    if (!inputFile) {
        std::cerr << "Unable to open file example.txt" << std::endl;
    }
    string line;
	int count = 0;
    // 逐行读取文件内容并输出到控制台
    while (getline(inputFile, line)) {
		// 检查并去除末尾的换行符
		if (!line.empty() && line[line.length() - 1] == '\n') {
			line.erase(line.length() - 1);
		}
		buffer[count] = line;
		if((++count)>=size){
			break;
		}
    }
    // 关闭文件
    inputFile.close();
}

void Txt_Input(string name, double *buffer, int size){
	ifstream inputFile(name);
    // 检查文件是否成功打开
    if (!inputFile) {
        std::cerr << "Unable to open file example.txt" << std::endl;
    }
    string line;
	int count = 0;
    // 逐行读取文件内容并输出到控制台
    while (getline(inputFile, line)) {
		// 检查并去除末尾的换行符
		if (!line.empty() && line[line.length() - 1] == '\n') {
			line.erase(line.length() - 1);
		}
		buffer[count] = stod(line);
		if((++count)>=size){
			break;
		}
    }
    // 关闭文件
    inputFile.close();
}

//C语言输出文件的方法////////////////////////////////////////////////////////////////////////////
//void writeExcel(void ) {
//	
//	FILE *fp = NULL;
//	int t;
//	char ch;
// 
//	fp = fopen("E:\\ALG_TEST_FILE\\test.xls", "w");
//	
//	for (int i = 0; i < 2; i++) {
//		printf("please input:");
//		scanf("%d %c", &t, &ch);
//		fprintf(fp, "%d\t%c\n", t, ch);
//	}
//	fclose(fp);
// 
//}

void file_create(string name, std::ios_base::openmode my_mode)//建立文件
{
	ofstream outfile(name, my_mode);
	if (!outfile)
	{
		cerr << "open error!" << endl;
		exit(1);
	}
	// cout << name << " ok" << endl;
	outfile.close();
}
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <stdio.h>
//#include <cstdlib>
#include "sqlite3.h"
#include <iostream>
#include <time.h>
#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <locale>

using namespace std;

class ConnectDB
{
public:
	ConnectDB(void);
	~ConnectDB(void);	

	//Chuẩn hóa id bảng Csm
	string normalizationIndexCsmrId(int index);

	//Connect database
	sqlite3 * connectDb(const char* path);

	//Chuyển đổi thời gian vào và ra 
	string convertTimeInOut(time_t t);

	//Lấy id mới nhất trong CustomerCD
	int getNewstIdCustomer(sqlite3* db, string soHieuCuaHang, time_t t);

	//Hàm tạo CAM_DATE
	string createCAM_DATE(time_t time);

	//Hàm tạo format date cho customer cd
	string createDateCustomerCd(time_t t);

	//Ghép định dạng CustomerId
	string createFormatCustomerId(string maCuaHang, string thoiGian, int index);

	//Chèn thêm bản ghi vào bảng TB_CAM_MARKET_CSMR
	bool insert_TB_CAM_MARKET_CSMR(sqlite3* db, string maCuaHang, string thoiGianCamDate, string maKhach, string timeIn, string timeOut, int timeOn);

	//Lấy thong tin camera
	string getCameraInfo(sqlite3* db);	

	//Lấy thong tin cửa hàng
	string getShopInfo(sqlite3* db);	

	string getCAM_DATE(sqlite3* db);
};


#include "stdafx.h"
#include "ConnectDB.h"

ConnectDB::ConnectDB(void)
{
}


ConnectDB::~ConnectDB(void)
{

}

static int callback(void *data, int argc, char **argv, char **azColName){
	int i;
	fprintf(stderr, "%s: ", (const char*)data);
	for(i=0; i<argc; i++){
		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}

string ConnectDB::normalizationIndexCsmrId(int index)
{
	char intstr[6];
	_itoa_s(index, intstr, 10);
	//itoa(index, intstr, 10);
	int str_length = string(intstr).length();
	char temp[6];
	for(int i = 0; i < 5; i++){
		if(i >= 5 - str_length) {
			int j = i - (5 - str_length);
			temp[i] = intstr[j]; 
		} else {
			temp[i] = '0';
		}
	}
	temp[5] = '\0';
	string id = string(temp);
	return id;
}

string ConnectDB::createCAM_DATE(time_t t)
{
	struct tm timeinfo;
	localtime_s(&timeinfo, &t); //convert it to tm
	//timeinfo = localtime(&t); //convert it to tm

	char t_CAM_DATE[10];
	const char c_CamDate[] = "%Y%m%d";

	if (strftime(t_CAM_DATE, sizeof(t_CAM_DATE) - 1, c_CamDate, &timeinfo)>0) {
		return string(t_CAM_DATE);
	}
	else
		return NULL;
}

sqlite3* ConnectDB::connectDb(const char* path)
{
	sqlite3 *db;
	char *zErrMsg = 0;
	int rc;
	rc = sqlite3_open(path, &db);
	if(rc){
		printf("SQL err: %d-- %s\n", rc, zErrMsg);
		sqlite3_close(db);
		return NULL;
	}
	else
	{
		printf("Opened successful! \n");   
		return db;
	}
}

string ConnectDB::convertTimeInOut(time_t t)
{
	struct tm timeinfo;
	localtime_s(&timeinfo, &t); //convert it to tm
	//timeinfo = localtime(&t); //convert it to tm
	char t_IN[21];
	const char c_InOut[]="%Y-%m-%d %H:%M:%S";

	if (strftime(t_IN, sizeof(t_IN)-1, c_InOut, &timeinfo)>0) {
		cout<<t_IN<< endl;
		return string(t_IN);
	}
	else 
		return NULL;
}
string ConnectDB::createDateCustomerCd(time_t t)
{
	struct tm timeinfo;
	localtime_s(&timeinfo, &t); //convert it to tm
	//timeinfo = localtime(&t); //convert it to tm
	char t_prefix[6];
	const char c[] = "%y%m";

	if (strftime(t_prefix, sizeof(t_prefix) - 1, c, &timeinfo)>0) {

		return string(t_prefix);
	}
	else
		return NULL;
}


int ConnectDB::getNewstIdCustomer(sqlite3* db, string soHieuCuaHang, time_t t)
{

	struct tm timeinfo;
	localtime_s(&timeinfo, &t); //convert it to tm
	//timeinfo = localtime(&t); //convert it to tm
	char time[8];
	const char c[] = "%Y%m";
	strftime(time, sizeof(time) - 1, c, &timeinfo);
	string time_in(time);
	//time_in.append("-1");

	char *sql = new char[500];
	sqlite3_stmt *statement;    
	//sprintf(sql, "select CAM_CSMR_CD from TB_CAM_MARKET_CSMR where SHOP_CD = '%s' and TIME_IN >= '%s';", soHieuCuaHang.c_str(), time_in.c_str());
	sprintf_s(sql,500, "select max(CAM_CSMR_CD) from TB_CAM_MARKET_CSMR where SHOP_CD = '%s' and substr(CAM_DATE,1,6) = '%s';", soHieuCuaHang.c_str(), time_in.c_str());
	//cout << sql <<endl;
	int current_index = 0;

	if ( sqlite3_prepare(db, sql, -1, &statement, 0 ) == SQLITE_OK ) {
		int res = 0;
		while ( 1 ) {
			res = sqlite3_step(statement);
			if ( res == SQLITE_ROW && sqlite3_column_text(statement, 0) != NULL ) {          
				string s = (char*)sqlite3_column_text(statement, 0);
				s = s.substr(9,5);
				int s_int = atoi(s.c_str());
				if(s_int >= current_index) current_index = s_int;
			} 
			if ( res == SQLITE_DONE || res==SQLITE_ERROR) {
				break;
			}    
		}
	}
	return current_index + 1;
}

string ConnectDB::getCameraInfo(sqlite3* db)
{
	string url = "";
	char *sql = new char[500];
	sqlite3_stmt *statement;    
	sprintf_s(sql,500, "SELECT VIDEO_PATH FROM TB_CAM_INFO;");
	//cout << sql <<endl;
	int current_index = 0;

	if ( sqlite3_prepare(db, sql, -1, &statement, 0 ) == SQLITE_OK ) {
		int res = 0;
		while ( 1 ) {
			res = sqlite3_step(statement);
			if ( res == SQLITE_ROW && sqlite3_column_text(statement, 0) != NULL) {          
				url = (char*)sqlite3_column_text(statement, 0);				
			} 
			if ( res == SQLITE_DONE || res==SQLITE_ERROR) {
				break;
			}    
		}
	}
	return url;
}

string ConnectDB::getShopInfo(sqlite3* db)
{
	string shopCd = "";
	char *sql = new char[500];
	sqlite3_stmt *statement;    
	sprintf_s(sql,500, "SELECT SHOP_CD FROM TB_CAM_INFO;");
	//cout << sql <<endl;
	int current_index = 0;

	if ( sqlite3_prepare(db, sql, -1, &statement, 0 ) == SQLITE_OK ) {
		int res = 0;
		while ( 1 ) {
			res = sqlite3_step(statement);
			if ( res == SQLITE_ROW && sqlite3_column_text(statement, 0) != NULL) {          
				shopCd = (char*)sqlite3_column_text(statement, 0);
			} 
			if ( res == SQLITE_DONE || res==SQLITE_ERROR) {
				break;
			}    
		}
	}
	return shopCd;
}

string ConnectDB::createFormatCustomerId(string maCuaHang, string thoiGian, int index)
{
	string current_index_string = normalizationIndexCsmrId(index);  // có dạng 00004
	printf("(%s)", current_index_string.c_str());
	string CSMR_ID;
	CSMR_ID.append(maCuaHang);
	CSMR_ID.append(thoiGian);
	CSMR_ID.append(current_index_string);
	cout << CSMR_ID << endl;

	return CSMR_ID;
}

bool ConnectDB::insert_TB_CAM_MARKET_CSMR(sqlite3* db, string maCuaHang, string thoiGianCamDate, string maKhach, string timeIn, string timeOut, int timeOn)
{
	char *sql = new char[500];
        sqlite3_stmt *statement;
	/*sprintf(sql, 
		"INSERT INTO TB_CAM_MARKET_CSMR (SHOP_CD , CAM_DATE, CAM_CSMR_CD, TIME_IN, TIME_OUT, TIME_IN_SHOP, SYNCHRONIZED)VALUES ('%s','%s', '%s', '%s', '%s', '%d', '0');", 
		SHOP_CD, thoiGianCamDate.c_str(), maKhach.c_str(), timeIn.c_str(), timeOut.c_str(), timeOn); */
		sprintf_s(sql,500, 
		"INSERT INTO [TB_CAM_MARKET_CSMR] (SHOP_CD , CAM_DATE, CAM_CSMR_CD, TIME_IN, TIME_OUT, TIME_IN_SHOP, SYNCHRONIZED)VALUES ('%s','%s', '%s', '%s', '%s', '%d', '0');", 
		maCuaHang.c_str(), thoiGianCamDate.c_str(), maKhach.c_str(), timeIn.c_str(), timeOut.c_str(), timeOn); 
        char *zErrMsg = 0;
        const char* data = "Callback function called";
        //if ( sqlite3_exec(db, sql, callback, (void*)data, &zErrMsg) == SQLITE_OK ) 
		int rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
		if(rc == SQLITE_OK || rc == SQLITE_DONE) 
		{
			//printf("insert OK\n");
			return true;
		}
        
	return false;
}

string ConnectDB::getCAM_DATE(sqlite3* db){
	char *sql = new char[500];
    sqlite3_stmt *statement;

	sprintf_s(sql, 500, "SELECT MAX(CAM_DATE) FROM TB_CAM_MARKET_CSMR;");
	string camdate = "";
	if ( sqlite3_prepare(db, sql, -1, &statement, 0 ) == SQLITE_OK ) {
		int res = 0;
		while ( 1 ) {
			res = sqlite3_step(statement);
			if ( res == SQLITE_ROW && sqlite3_column_text(statement, 0) != NULL) {          
				camdate = (char*)sqlite3_column_text(statement, 0);
			} 
			if ( res == SQLITE_DONE || res==SQLITE_ERROR) {
				break;
			}    
		}
	}
	return camdate;
}
#include "stdafx.h"
#include "CommandControl.h"
#include "Communication.h"

#include <stdio.h>
#include <time.h>

char Communication::PackLen[10] = { 0, 13, 13, 13, 13, 17, 13, 0, 0, 37 };

int Communication::ClearComm()
{
    if (Comm == NULL || Comm->BytesToRead() == 0)
    {
        return 0;
    }

	int len = Comm->BytesToRead();
    BYTE* buffer = new BYTE[len];
    Comm->Read(buffer, len);    
	delete[] buffer;

	return len;
}

BOOL Communication::WritePack(BYTE* buff, int len)
{
    buff[len - 2] = CRC8::GetParity(buff, len - 2);
    try
    {
        Comm->Write(buff, len);
    }
	catch(std::exception& exp)
    {
        return FALSE;
    }

    PrintPack(buff, len);

    return TRUE;
}

BOOL Communication::ReadPack(PackType readingPackType, BYTE* buff, char* errmsg)
{
    try
    {
		// 是否有足够的数据
		int timeCounter = 0;
		int blen = Comm->BytesToRead();
		if(blen<PackLen[readingPackType] && readingPackType!=LongData)
		{
			while(blen<PackLen[readingPackType])
			{
				if(timeCounter*50>3000) // 3秒
				{
					sprintf(errmsg, "%s", "等待超时");
					return FALSE;
				}
				Sleep(50);
				timeCounter++;

				blen = Comm->BytesToRead();
			}
		}
		
        // 重试5次
        int repiteCount = 5;
ReadRepeate:
        if (repiteCount == 0)
        {
			sprintf(errmsg, "%s", "5次尝试读取数据失败");
            return FALSE;
        }
        repiteCount--;

        //time_t stm1;
        //time(&stm1);

        clock_t t = clock();
        double stm1 = (t/CLOCKS_PER_SEC);
FindePack:

		// 是否超时
        //time_t stm2;
        //time(&stm2);
        //struct tm* timeInfo1 = localtime(&stm1);
        //struct tm* timeInfo2 = localtime(&stm2);
        t = clock();
        double stm2 = (t/CLOCKS_PER_SEC);
        // time span
        //int timeSpan = timeInfo2->tm_sec - timeInfo1->tm_sec;
        int timeSpan = stm2 - stm1;
		if(timeSpan>20)
		{
            sprintf(errmsg, "%s", "Error: Read timeout");
			return FALSE;
		}

        int dataBufferPos = 0;

        // read 0xfd
        if (-1 == Comm->Read(buff + dataBufferPos, 1))
        {
            goto ReadRepeate;
        }
        if (buff[dataBufferPos] != 0xfd)
        {
            goto FindePack;
        }
        dataBufferPos += 1;

        // read frame type
        if (-1 == Comm->Read(buff + dataBufferPos, 1))
        {
            goto ReadRepeate;
        }
        if (buff[dataBufferPos] < 1 || buff[dataBufferPos] > 6)
        {
            goto FindePack;
        }
        dataBufferPos += 1;

        int frmLen = PackLen[buff[1]];
        if (buff[1] != 0x04)
        {
            // read sub data
            if (-1 == Comm->Read(buff + dataBufferPos, frmLen - 2))
            {
                goto ReadRepeate;
            }
        }
        else
        {
            // read pack info
            if (-1 == Comm->Read(buff + dataBufferPos, 5))
            {
                goto ReadRepeate;
            }
            dataBufferPos += 5;

            frmLen = buff[6];

            // read sub data
            if (-1 == Comm->Read(buff + dataBufferPos, frmLen - 7))
            {
                goto ReadRepeate;
            }
        }

        if (buff[frmLen - 1] != 0xfe)
        {
            //errmsg = "包校验错误：包尾不正确";
            goto FindePack;
        }

		if (buff[frmLen-2]!=CRC8::GetParity(buff, frmLen-2))
        {
			sprintf(errmsg, "%s", "包校验错误");
            PrintPack(buff, frmLen);
            return FALSE;
        }

        PrintPack(buff, frmLen);
                
        BOOL checkResult = FALSE;

        switch (buff[1])
        {
            case 0x01:
				sprintf(errmsg, "%s", "错误：上位机收到命令帧\n");
				strcat(errmsg, "错误：下位机已进入睡眠状态，请激活");
				return FALSE;
                break;
            case 0x02:
                checkResult = CheckResponse(buff, frmLen, errmsg);
                break;
            case 0x03:
                checkResult = CheckResult(buff, frmLen, errmsg);
                break;
            case 0x04:
                checkResult = CheckLongData(buff, frmLen, errmsg);
                break;
            case 0x05:
                checkResult = CheckShortData(buff, errmsg);
                break;
            case 0x06:
                checkResult = CheckWarning(buff, errmsg);
                break;
            default:
                sprintf(errmsg, "%s", "错误：未知帧类型");
                break;
        }

        // 读取到的不是所要的包，丢弃
        if (readingPackType != (PackType)buff[1])
        {
            goto FindePack;
        }

        return checkResult;
    }
	catch (std::exception& ex)
    {
		sprintf(errmsg, "读取数据包错误：%s", ex.what());
    }

    return FALSE;
}

void Communication::PrintPack(BYTE* buff, int len)
{
	int bl = len*3+1;
	if(bl<=0) return;

    for (int i = 0; i < len; i++)
    {
        printf("%02X ", buff[i]);
    }
}


BOOL Communication::CheckResponse(BYTE* buff, int len, char* errmsg)
{
    if (buff[1] != 0x02)
    {
        return FALSE;
    }
	
	sprintf(errmsg, "%s", "应答：");

    if (len < 13)
    {
        strcat(errmsg, "应答帧长度错误");
        return FALSE;
    }

    switch (buff[10])
    {
        case 0x0:
            strcat(errmsg, "待处理");
            return TRUE;
        case 0x1:
            strcat(errmsg, "收到1条校验错误的命令帧");
			break;
        case 0x2:
            strcat(errmsg, "帧头、帧尾、或帧类型不正确");
			break;
        case 0x3:
            strcat(errmsg, "系统忙");
			break;
        case 0x4:
            strcat(errmsg, "保留");
			break;
        case 0x5:
            strcat(errmsg, "LSR错误数据");
			break;
        default:
            strcat(errmsg, "未知的应答帧类型");
			break;
    }
    return FALSE;
}

BOOL Communication::CheckResult(BYTE* buff, int len, char* errmsg)
{
    if(buff[1]!=0x03)
    {
        return FALSE;
    }
	
	sprintf(errmsg, "%s", "执行结果：");

    if(len<13)
    {
        strcat(errmsg, "结果帧长度错误");
        return FALSE;
    }

    if (buff[8] == 0 && buff[9] == 0 && buff[10] == 0)
    {
        strcat(errmsg, "正确");
        return TRUE;
    }
            
    if (buff[8] == 1)
    {
        switch (buff[9])
        {
            case 0x1:
                strcat(errmsg, "执行条件不满足");
                break;
            case 0x2:
                strcat(errmsg, "校准错误");
                break;
            case 0x3:
                strcat(errmsg, "EEPROM读写失败");
                break;
            case 0x4:
                strcat(errmsg, "命令参数错误");
                break;
            default:
                strcat(errmsg, "未知的结果帧类型");
                break;
        }
    }
    else if (buff[8] == 2)
    {
        switch (buff[9])
        {
            case 0x1:
                strcat(errmsg, "查询条件不满足");
                break;
            case 0x2:
                strcat(errmsg, "保留");
                break;
            case 0x3:
                strcat(errmsg, "EEPROM读写失败");
                break;
            case 0x4:
                strcat(errmsg, "命令参数错误");
                break;
            default:
                strcat(errmsg, "未知的结果帧类型");
                break;
        }
    }
    else if (buff[8] == 3)
    {
        switch (buff[9])
        {
            case 0x1:
                strcat(errmsg, "配置条件不满足");
                break;
            case 0x2:
                strcat(errmsg, "保留");
                break;
            case 0x3:
                strcat(errmsg, "EEPROM读写失败");
                break;
            case 0x4:
                strcat(errmsg, "命令参数错误");
                break;
            default:
                strcat(errmsg, "未知的结果帧类型");
                break;
        }
    }
    else
    {
        strcat(errmsg, "未知错误");
    }
	return FALSE;
}

BOOL Communication::CheckLongData(BYTE* buff, int len, char* msg)
{
    if(buff[1]!=0x04)
    {
        return FALSE;
    }

    int dataLen = buff[6]-7;
    int dataPos = 7;
	
	sprintf(msg, "%s", "返回数据：");

	char tmp[200];
	memset(tmp, '\0', sizeof(tmp));

    switch (buff[5]) // 数据类型
    {
        case 0x00:
			sprintf(tmp, "中位机固件版本：%d.%d.%d", buff[dataPos + 2], buff[dataPos + 1], buff[dataPos + 0]);
            strcat(msg, tmp);
            break;
        case 0x01:
			{
				int MainVer = (buff[dataPos + 1] >> 4);
				int SubVer  = (buff[dataPos + 0] >> 4);
				int CtrlVer = (buff[dataPos + 0] & 0x0f);
				char tmp[200];
				memset(tmp, '\0', sizeof(tmp));
				sprintf(tmp, "下位机固件版本：%d.%d.%d", MainVer, SubVer, CtrlVer);
				strcat(msg, tmp);
			}
            break;
        case 0x02:
            strcat(msg, "陀螺仪AD零点偏置：\n");
				
			sprintf(tmp, "x=%d\ny=%d\n%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
            strcat(msg, tmp);

            strcat(msg, "加速度校准系数矩阵：\n");
			memset(tmp, '\0', sizeof(tmp));
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 6] + (buff[dataPos + 7] << 8)), (short)(buff[dataPos + 8] + (buff[dataPos + 9] << 8)), (short)(buff[dataPos + 10] + (buff[dataPos + 11] << 8)));
            strcat(msg, tmp);
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 12] + (buff[dataPos + 13] << 8)), (short)(buff[dataPos + 14] + (buff[dataPos + 15] << 8)), (short)(buff[dataPos + 16] + (buff[dataPos + 17] << 8)));
            strcat(msg, tmp);
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 18] + (buff[dataPos + 19] << 8)), (short)(buff[dataPos + 20] + (buff[dataPos + 21] << 8)), (short)(buff[dataPos + 22] + (buff[dataPos + 23] << 8)));
            strcat(msg, tmp);
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 24] + (buff[dataPos + 25] << 8)), (short)(buff[dataPos + 26] + (buff[dataPos + 27] << 8)), (short)(buff[dataPos + 28] + (buff[dataPos + 29] << 8)));
            strcat(msg, tmp);

            strcat(msg, "电子罗盘校准系数：\n");
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 30] + (buff[dataPos + 31] << 8)), (short)(buff[dataPos + 32] + (buff[dataPos + 33] << 8)), (short)(buff[dataPos + 34] + (buff[dataPos + 35] << 8)));
            strcat(msg, tmp);
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 36] + (buff[dataPos + 37] << 8)), (short)(buff[dataPos + 38] + (buff[dataPos + 39] << 8)), (short)(buff[dataPos + 40] + (buff[dataPos + 41] << 8)));
            strcat(msg, tmp);
            break;
        case 0x03:
            strcat(msg, "中位机产品信息：\n");
			sprintf(tmp, "生产流水号： %d\n", buff[dataPos + 0] + (buff[dataPos + 1] << 8));
            strcat(msg, tmp);
			sprintf(tmp, "    生产周： %d\n", buff[dataPos + 2]);
            strcat(msg, tmp);
			sprintf(tmp, "  生产年份： %d\n", buff[dataPos + 3]);
            strcat(msg, tmp);
            break;
        case 0x04:
            strcat(msg, "下位机产品信息：\n");
			sprintf(tmp, "生产流水号： %d\n", buff[dataPos + 0] + (buff[dataPos + 1] << 8));
            strcat(msg, tmp);
			sprintf(tmp, "    生产周： %d\n", buff[dataPos + 2]);
            strcat(msg, tmp);
			sprintf(tmp, "  生产年份： %d\n", buff[dataPos + 3]);
            strcat(msg, tmp);
			//sprintf(tmp, "  项目代号： %s\n", buff[dataPos + 4]==0x86?"MCP":"GOLF");
            //strcat(msg, tmp);
            break;
        case 0x05:
            strcat(msg, "<控制参数>");
            break;
        case 0x06:
            strcat(msg, "陀螺仪AD值：\n");
			sprintf(tmp, "x=%d\ny=%d\nx=%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
            strcat(msg, tmp);
            break;
        case 0x07:
            if (DevType == MCP)
            {
                strcat(msg, "加速度AD值：\n");
				sprintf(tmp, "x=%d\ny=%d\nx=%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
				strcat(msg, tmp);
                    
            }
            else if (DevType == Golf)
            {
                strcat(msg, "加速度AD值：\n");
				sprintf(tmp, "x=%d\ny=%d\nx=%d\n", (int32_t)(buff[dataPos + 0] + (buff[dataPos + 1] << 8) + (buff[dataPos + 2] << 16) + (buff[dataPos + 3] << 24)),
                    (int32_t)(buff[dataPos + 4] + (buff[dataPos + 5] << 8) + (buff[dataPos + 6] << 16) + (buff[dataPos + 7] << 24)),
                    (short)(buff[dataPos + 8] + (buff[dataPos + 9] << 8)));
				strcat(msg, tmp);                        
            }
            break;
        case 0x08:
            strcat(msg, "电子罗盘AD值：\n");
			sprintf(tmp, "x=%d\ny=%d\nx=%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
			strcat(msg, tmp);
            break;
        case 0x09:
            strcat(msg, "陀螺仪校准值：\n");
			sprintf(tmp, "x=%d\ny=%d\nx=%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
			strcat(msg, tmp);
            break;
        case 0x0a:
            strcat(msg, "加速度校准系数矩阵：\n");

            UnionFloat fv;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    fv.buff[0] = buff[dataPos + (i * 12) + (j * 4) + 0];
                    fv.buff[1] = buff[dataPos + (i * 12) + (j * 4) + 1];
                    fv.buff[2] = buff[dataPos + (i * 12) + (j * 4) + 2];
                    fv.buff[3] = buff[dataPos + (i * 12) + (j * 4) + 3];
					sprintf(tmp, "%0.4f\t\t", fv.value);
					strcat(msg, tmp);
                }
                strcat(msg, "\n");
            }
            break;
        case 0x0b:
            strcat(msg, "电子罗盘校准系数：\n");
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)), (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)), (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
			strcat(msg, tmp);
			sprintf(tmp, "%d\t%d\t%d\n", (short)(buff[dataPos + 6] + (buff[dataPos + 7] << 8)), (short)(buff[dataPos + 8] + (buff[dataPos + 9] << 8)), (short)(buff[dataPos + 10] + (buff[dataPos + 11] << 8)));
			strcat(msg, tmp);
            break;
        case 0x0c:
			sprintf(tmp, "中位机工作模式：%s", (buff[dataPos + 0]==0x00?"数据模式":"命令模式"));
			strcat(msg, tmp);
            break;
        case 0x0d:
            strcat(msg, "中位机内“下位机ID”备份：\n");
            strcat(msg, "<数据未分解>");
            break;
        case 0x0e:
            strcat(msg, "“中位机射频发送地址低”字节\n");
            strcat(msg, "<数据未分解>");
            break;
        case 0x0f:
            strcat(msg, "电池状态\n");
            strcat(msg, "<数据未分解>");
            break;
        case 0x10:
            strcat(msg, "温度：");
			sprintf(tmp, "%d摄氏度", (short)(buff[dataPos + 0]));
			strcat(msg, tmp);
            break;
        case 0x20:
            strcat(msg, "欧拉角：\n");
			sprintf(tmp, " roll=%d\n", (short)(buff[dataPos + 0] + (buff[dataPos + 1] << 8)));
			strcat(msg, tmp);
			sprintf(tmp, "pitch=%d\n", (short)(buff[dataPos + 2] + (buff[dataPos + 3] << 8)));
			strcat(msg, tmp);
			sprintf(tmp, "  yaw=%d\n", (short)(buff[dataPos + 4] + (buff[dataPos + 5] << 8)));
			strcat(msg, tmp);
            break;
        case 0x21:
            strcat(msg, "传感器原始数据\n");
            strcat(msg, "<数据未分解>");
            break;
        case 0x31:
            strcat(msg, "Golf应用单节点挥杆识别触发数据帧：\n");
            strcat(msg, "<数据未分解>");
            break;
        case 0x70:
            {
                strcat(msg, "有未能启动采集的子节点，ID分别为：\n");

                char ids[200];
				memset(ids, 0, sizeof(ids));

                int pos = 7;
                int len = buff[6] - 2;
                while (pos < len)
                {
                    int sensorId = buff[pos];
                    if (strlen(ids) != 0)
                    {
                        strcat(ids, " ");
                    }
					char hex[20];
					sprintf_s(hex, sizeof(hex), "%02X", sensorId);
                    strcat(ids, hex);

                    pos += 1;
                }

                strcat(msg, ids);
            }
            break;
        case 0x71:
            {
                strcat(msg, "版本号：\n");
                int pos = 7;
                int len = buff[6] - 2;
                while (pos < len)
                {
                    int moduleId = buff[pos];
                    int verSub = buff[pos + 1] >> 4;
                    int verRev = (buff[pos + 1] << 4) >> 4;
                    int verMain = buff[pos + 2] >> 4;
                    //int verTime = (buff[pos + 2] << 4) >> 4;
					sprintf(tmp, "%d.%d.%d.%d\n", moduleId, verMain, verSub, verRev);
					strcat(msg, tmp);
                    pos += 3;
                }
            }
            break;
        case 0x72:
            {
                strcat(msg, "电量：\n");
                int pos = 7;
                int len = buff[6] - 2;
                while (pos < len)
                {
                    int moduleId = buff[pos];
                    int level = buff[pos + 2] >> 5;
					sprintf(tmp, "%d:%d\n", moduleId, level);
					strcat(msg, tmp);
                    pos += 3;
                }
            }
            break;

        default:
            strcat(msg, "未知的数据类型");
            return FALSE;
    }

    return TRUE;
}
BOOL Communication::CheckShortData(BYTE* buff, char* errmsg)
{
	sprintf(errmsg, "%s", "数据未分解");

    return FALSE;
}
BOOL Communication::CheckWarning(BYTE* buff, char* msg)
{
    if (buff[1] != 0x06)
    {
        return FALSE;
    }
		
	sprintf(msg, "%s", "警告：");

    switch (buff[8]) // 索引号
    {
        case 0x01:
            strcat(msg, "客户校准参数1");
            break;
        case 0x02:
            strcat(msg, "工厂校准参数1");
            break;
        case 0x03:
            strcat(msg, "系统配置参数");
            break;
        case 0x04:
            strcat(msg, "控制参数");
            break;
        case 0x05:
            strcat(msg, "转发");
            break;
        case 0x06:
            strcat(msg, "客户校准参数2");
            break;
        case 0x07:
            strcat(msg, "工厂校准参数2");
            break;
        case 0x08:
            strcat(msg, "按钮触发");
            break;
        case 0x09:
			{
				strcat(msg, "电量状态：\n");

				char tmp[200];
				memset(tmp, '\0', sizeof(tmp));
				sprintf(tmp, "电压=%dV\n", (int)((((buff[10] & 0x1f) << 8) + buff[9]) / 1000.0));
                				
				strcat(msg, tmp);
                
				strcat(msg, "级别=");

				int level = (buff[10] & 0xe0) >> 5;
				switch (level)
				{
					case 0x00:
						strcat(msg, "低于等于3.7V，低电量报警");
						break;
					case 0x01:
						strcat(msg, "3.7V~3.8V，充电提示");
						break;
					case 0x02:
						strcat(msg, "3.8V~3.9V，电量正常");
						break;
					case 0x03:
						strcat(msg, "3.9V~4.3V，满电量");
						break;
					case 0x04:
					case 0x05:
					case 0x06:
						strcat(msg, "保留");
						break;
					case 0x07:
						strcat(msg, "异常");
						break;
					default:
						strcat(msg, "未知级别");
						break;
				}
			}
            break;
        case 0x10:
            strcat(msg, "命令未定义");
            break;
        default:
            strcat(msg, "未知警告类型");
            return FALSE;
    }

    // 细节1
    char detail1[500];
	memset(detail1, '\0', sizeof(detail1));
    CheckWarningDetail1(buff, detail1);
    if (memcmp(detail1 , "", strlen(detail1))!=0)
    {
        strcat(msg, "\n");
        strcat(msg, detail1);
    }

    // 细节2
    char detail2[500];
	memset(detail2, '\0', sizeof(detail2));
    CheckWarningDetail2(buff, detail2);
    if (memcmp(detail2 , "", strlen(detail2))!=0)
    {
        strcat(msg, "\n");
        strcat(msg, + detail2);
    }

    return TRUE;
}
    
void Communication::CheckWarningDetail1(BYTE* buff, char* msg)
{
    if (buff[1] != 0x06)
    {
        return;
    }
		
	sprintf(msg, "%s", "细节1：");

    switch (buff[9]) // 细节1
    {
        case 0x01:
            strcat(msg, "无意义");
            break;
        case 0x02:
            strcat(msg, "参数无效");
            break;
        case 0x03:
            strcat(msg, "EEPROM读写错误");
            break;
        case 0x04:
            strcat(msg, "抄送地址不匹配");
            break;
        case 0x05:
            strcat(msg, "抄送失败");
            break;
        default:
            strcat(msg, "未知的细节1");
            break;
    }
}
void Communication::CheckWarningDetail2(BYTE* buff, char* msg)
{
    if (buff[1] != 0x06)
    {
        return;
    }

	sprintf(msg, "%s", "细节2：");

    switch (buff[10]) // 细节2
    {
        case 0x01:
            strcat(msg, "无意义");
            break;
        case 0x02:
            strcat(msg, "陀螺仪参数");
            break;
        case 0x03:
            strcat(msg, "电子罗盘参数");
            break;
        case 0x04:
            strcat(msg, "产品ID");
            break;
        case 0x05:
            strcat(msg, "下位机接收地址");
            break;
        case 0x06:
            strcat(msg, "下位机接收地址有效标志");
            break;
        case 0x07:
            strcat(msg, "客户校准参数有效标志");
            break;
        case 0x08:
            strcat(msg, "工厂校准参数有效标志");
            break;
        case 0x09:
            strcat(msg, "控制参数有效标志");
            break;
        default:
            strcat(msg, "未知的细节2");
            break;
    }
}

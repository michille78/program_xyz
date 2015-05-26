// ControlDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "postrack.h"
#include "ControlDlg.h"
#include "afxdialogex.h"

#include "PosTrackor.h"
#include "CommandControl.h"
#include "serialport.h"

CRITICAL_SECTION g_cs;

// 光学数据
void CALLBACK recievePosTrackor(void * pOwner, float* optiTracData)
{
    if (pOwner == NULL) return ;

    CControlDlg* dlg = (CControlDlg*)pOwner;    
    if (dlg == NULL)return ;

	// save optic track data for data fusion
	dlg->OptiTracData = Point3D_t(optiTracData[0], optiTracData[2], optiTracData[1]); // 转成惯性坐标系
}

// 光学数据
void CALLBACK recieveOherMark(void * pOwner, float* otherMarkData, int count)
{
	if (pOwner == NULL) return;
	CControlDlg* dlg = (CControlDlg*)pOwner;
	if (dlg == NULL)return;

	if (!dlg->isWriteRawFile) return;

	SYSTEMTIME  time;
	GetLocalTime(&time);

	if (dlg->fpOpt)
	{
		fprintf(dlg->fpOpt, "%02d:%02d:%02d    %d", time.wMinute, time.wSecond, time.wMilliseconds, count);
		if (count)
		{
			for (int i = 0; i < count; i++)
			{
				fprintf(dlg->fpOpt, " %0.5f %0.5f %0.5f",  
					 otherMarkData[i + 0], otherMarkData[i + 1], otherMarkData[i + 2]);
			}
		}
		fprintf(dlg->fpOpt, "\n");
	}

}

void __stdcall CalculatedBinaryDataCallback(void* customObject, int avatarIndex, CalculationDataHeader* cbp, int packLen)
{
   //中间默认输出的QAW为 QT_GlobalRawQuat AT_ModelRawData GY_ModelRawData
    
    CControlDlg* dlg = (CControlDlg*)customObject;
    SYSTEMTIME  time;
    GetLocalTime(&time);

	// Get X data
    float X[126] = { 0.0 };
    float* begin = (float*)((char*)cbp + sizeof(CalculationDataHeader));
	for (int i = 0; i < 21; i++)
	{
		for (int j = 0; j < 6;j++)
		{
			X[i * 6 + j] = begin[i * 16 + j];
		}
	}
	
	if (dlg->isWriteRawFile)
	{
		if (dlg->fpInertia)
		{
		fprintf(dlg->fpInertia, "%02d:%02d:%02d    "                   // time 分：秒：毫秒
			"%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f "               // hip-q hip-disp
			"%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f\n",             // head-q head-disp
			time.wMinute, time.wSecond, time.wMilliseconds,
			begin[6], begin[7], begin[8], begin[9], X[0], X[1], X[2],
			begin[246], begin[247], begin[248], begin[249], X[90], X[91], X[92]);
		}
	}

    // Save org head pos data
    sprintf(dlg->tmpstr, "%02d-%02d-%02d-%02d  %0.3f %0.3f %0.3f",
        time.wHour, time.wMinute, time.wSecond, time.wMilliseconds,
        X[90], X[91], X[92]); //  inert X Y Z 

    // Calibrating
    if (dlg->pPosTrackor->IsCalibrating)
    {
        Point3D_t pt;
        pt.X = X[90];
        pt.Y = X[91];
        pt.Z = X[92];
        dlg->m_wndMsg.SetWindowText(L"光学校准中...");
        dlg->pPosTrackor->AddCalibrationData(pt);
        return;
    }

    // 头部的节点是10 是第15根骨骼， 跳过前15*6 = 90(0~90)   90:x  91:y 92:z
    // 数据融合
    dlg->DataFusion(X, dlg->OptiTracData);

}

void CControlDlg::DataFusion(float* bonePosX, Point3D_t optiTracData)
{
	char tmpFileStr[300];
	memset(tmpFileStr, '\0', 300);

	// 是否添加校准数据原始校准数据写入文件中
	if (pPosTrackor->IsCalibrating)return;
	// 如果没有做过校准则返回
	if (!pPosTrackor->IsCalibrated)return;

	// Save tmp data
	Point3D_t tmpOrgOptiTrac = optiTracData;

	// Get head position data
	Point3D_t headPosition(bonePosX[90], bonePosX[91], bonePosX[92]);

	// 修正惯性系位移
	headPosition = headPosition - pPosTrackor->Trans1;

	// 将光学系平移到惯性系，使原点与惯性系重合
	optiTracData.X += pPosTrackor->Trans2.X;
	optiTracData.Y += pPosTrackor->Trans2.Y;
	optiTracData.Z += pPosTrackor->Trans2.Z;

	// 将光学系旋转到惯性系，使方向与惯性系重合
	Vector2d rotatedOptiTrac = pPosTrackor->R.transpose() * Vector2d(optiTracData.X, optiTracData.Y);
	optiTracData.X = rotatedOptiTrac[0];
	optiTracData.Y = rotatedOptiTrac[1];

	// 获取光学系与惯性系的差
	Point3D_t offset = headPosition - optiTracData;
	Point3D_t outputOffset = offset;

	// 纠偏系数
	static double rectifyFactor = 0.3;
	// 通过纠偏系数降低每次纠偏的量
	offset = offset * rectifyFactor;
	offset = offset * rectifyFactor;

	// 纠偏
	for (int i = 0; i < 126; i += 6)
	{
		bonePosX[i + 0] -= offset.X;
		bonePosX[i + 1] -= offset.Y;
		//bonePosX[i + 2] -= offset.Z;
	}

	// 应用到迭代中
	PNUpdateXt(0, bonePosX);

	m_Msg.Format(L"x = %0.3f, y = %0.3f, z = %0.3f \r\n r_x = %0.3f, y = %0.3f, r_z = %0.3f",
		tmpOrgOptiTrac.X, tmpOrgOptiTrac.Y, tmpOrgOptiTrac.Z,
		optiTracData.X, optiTracData.Y, optiTracData.Z);
	m_wndMsg.SetWindowText(m_Msg);

	if (!isWriteFile) return;

	// Time 原始惯性XYZ  偏移过的惯性XYZ  偏移过+旋转光学 XZY  原始光学XZY
	sprintf_s(tmpFileStr, "%s  %0.3f %0.3f %0.3f  %0.3f %0.3f %0.3f  %0.3f %0.3f %0.3f %0.3f\n",
		tmpstr,
		headPosition.X, headPosition.Y, headPosition.Z,
		optiTracData.X, optiTracData.Y, optiTracData.Z,
		tmpOrgOptiTrac.X, tmpOrgOptiTrac.Y, tmpOrgOptiTrac.Z,
		outputOffset.mod());

	if (fpPos == NULL) return;

	fwrite(tmpFileStr, strlen(tmpFileStr), 1, fpPos);
}


void __stdcall CalibrationProgressCallback(void* HandleObject, int avatarIndex, float percent)
{
    CControlDlg* dlg = (CControlDlg*)HandleObject;
    wchar_t str[20];
    swprintf_s(str,20, L"%d%%\0", (int)(percent*100));
    dlg->m_wndProgress.SetWindowText(str);
}

IMPLEMENT_DYNAMIC(CControlDlg, CDialogEx)

CControlDlg::CControlDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CControlDlg::IDD, pParent)
{
    pPosTrackor = NULL;
    
    fpPos = NULL;
    //fpCali = NULL;
	fpInertia = NULL;
	fpOpt = NULL;

    memset(tmpstr, '\0', 200);
    
    pSerial = NULL;

    pCommandControl = NULL;

    m_ThreadFlag = 1;
    
    hThread = NULL;

    memset(data, '\0', sizeof(data));

    isWriteFile = false;
	isWriteRawFile = false;

	qType = QT_GlobalBoneQuat;
}

CControlDlg::~CControlDlg()
{
    // 跳出读取线程
    m_ThreadFlag = 0;
    Sleep(1000);
    if (hThread)
    {
        CloseHandle(hThread);
        hThread = NULL;
    }
}

void CControlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_wndRatioList);
	DDX_Control(pDX, IDC_EDIT_IP, m_wndIP);
	DDX_Control(pDX, IDC_EDIT_PORT, m_wndPort);
	DDX_Control(pDX, IDC_STC_PROGRESS, m_wndProgress);
	DDX_Control(pDX, IDC_EDIT1, m_wndMsg);
	DDX_Control(pDX, IDC_STATIC_MSG, m_wndMessage);
	DDX_Control(pDX, IDC_COMBO1, wnd_qType);
}


BEGIN_MESSAGE_MAP(CControlDlg, CDialogEx)
    ON_WM_TIMER()
    ON_BN_CLICKED(IDC_BTN_TPOS, &CControlDlg::OnBnClickedBtnTpos)
    ON_BN_CLICKED(IDC_BTN_APOS, &CControlDlg::OnBnClickedBtnApos)
    ON_BN_CLICKED(IDC_BTN_SPOS, &CControlDlg::OnBnClickedBtnSpos)
    ON_BN_CLICKED(IDC_BTN_POS, &CControlDlg::OnBnClickedBtnPos)
    ON_BN_CLICKED(IDC_BTN_OPENPORT, &CControlDlg::OnBnClickedBtnOpenport)
    ON_BN_CLICKED(IDC_BTN_CLOSSENSOR, &CControlDlg::OnBnClickedBtnClossensor)
    ON_BN_CLICKED(IDC_BTN_CONNECT, &CControlDlg::OnBnClickedBtnConnect)
    ON_BN_CLICKED(IDC_BTN_DISCONNECT, &CControlDlg::OnBnClickedBtnDisconnect)
    ON_BN_CLICKED(IDC_BTN_WRITEFILE, &CControlDlg::OnBnClickedBtnWritefile)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BTN_WRITERAWFILE, &CControlDlg::OnBnClickedBtnWriterawfile)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CControlDlg::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(IDC_BUTTON_ZERO, &CControlDlg::OnBnClickedButtonZero)
END_MESSAGE_MAP()


// CControlDlg 消息处理程序


BOOL CControlDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // TODO:  在此添加额外的初始化
    // 设置得包率显示
    m_wndRatioList.InsertColumn(0, _T("BoneID"), LVCFMT_CENTER, 100);
    m_wndRatioList.InsertColumn(1, _T("Ratio"), LVCFMT_LEFT, 100);
    m_wndRatioList.SetExtendedStyle(LVS_EX_FULLROWSELECT|LVS_EX_GRIDLINES);

	// 输出的Q的初始化
	wnd_qType.SetCurSel(0);

    for (int i = 0; i < 17; i++)
    {
        TCHAR strTmp[5];
        swprintf_s(strTmp, 5, L"%d", i+1);
        m_wndRatioList.InsertItem(i, strTmp);
    }

    // 设置BoneQ的中间数据格式
	PNSetCalculatedQuaternionDataType(qType);

    // 开启实时中间数据
    PNEnableCalculationDataBoardcast(TRUE);

    // 设置校准进度回调函数
    PNRegisterCalibrationProgressHandle(this, CalibrationProgressCallback);

    // 设置中间数据回调函数
    PNRegisterCalculatedBinaryDataBoardcastHandle(this, CalculatedBinaryDataCallback);


    pPosTrackor = new PosTrackor();
    pPosTrackor->SetRecievePosTrackHandle(this, recievePosTrackor);
	pPosTrackor->SetRecieveOtherMarkHandle(this, recieveOherMark);
    // 设置连接默认值
    m_wndIP.SetWindowText(L"127.0.0.1");
    m_wndPort.SetWindowText(L"1511");

    // 定时器获取传感器状态
    SetTimer(1, 1000, NULL);
    
    // 串口管理的
    pSerial = new CSerialPort();
    pCommandControl = new SerialControl(pSerial);

    // 创建一个惯性采集线程
    hThread = CreateThread(NULL, 0, Acquisition, this, 0, NULL);

    return TRUE;  // return TRUE unless you set the focus to a control
    // 异常: OCX 属性页应返回 FALSE
}


BOOL CControlDlg::PreTranslateMessage(MSG* pMsg)
{
    // TODO: 在此添加专用代码和/或调用基类
    if(pMsg->message==WM_KEYDOWN && pMsg->wParam==VK_ESCAPE)  return TRUE;
    if(pMsg->message==WM_KEYDOWN && pMsg->wParam==VK_RETURN) return TRUE; 
    return CDialogEx::PreTranslateMessage(pMsg);
}


void CControlDlg::OnTimer(UINT_PTR nIDEvent)
{
    // TODO: 在此添加消息处理程序代码和/或调用默认值
    switch (nIDEvent)
    {
    case 1:
        {
            double SensorRatios[167] = {0.0};
            UINT SensorState[167];

            // 得到传感器的Ratios
            PNGetSensorReceivingStatus(0, SensorRatios);

            LVITEM lvitem;
            lvitem.mask = LVIF_TEXT;
            lvitem.iItem = 0;
            lvitem.iSubItem = 1;
            wchar_t str[10];
            for (int i = 0; i < 19; i++)
            {
                SensorState[i] = UINT(SensorRatios[i]*100.0f);			
                swprintf_s(str, 6, _T("%d%%"), SensorState[i]);
                if (i == 17 || i == 16 ) continue;
                if (i == 18)
                {
                    m_wndRatioList.SetItemText(16, 1, str);
                }
                else
                {
                    m_wndRatioList.SetItemText(i, 1, str);
                }
                
            }
        }
        break;
    default:
        break;
    }
    CDialogEx::OnTimer(nIDEvent);
}


void CControlDlg::OnBnClickedBtnTpos()
{
    // TPos
    Sleep(3000);
    PNCalibrateAllAvatars(Cali_TPose);
}


void CControlDlg::OnBnClickedBtnApos()
{
    // APos
    Sleep(3000);
    PNCalibrateAllAvatars(Cali_APose);
}


void CControlDlg::OnBnClickedBtnSpos()
{
    // SPos
    Sleep(3000);
    PNCalibrateAllAvatars(Cali_SPose);
}

// 光学校准
void CControlDlg::OnBnClickedBtnPos()
{
    
    if (pPosTrackor->IsEnabled())
    {
   
        pPosTrackor->SetToStartCalibration();
    }
    else
    {
         MessageBox(L"光学设备未连接！");
    }
}


void CControlDlg::OnBnClickedBtnOpenport()
{
    if (pSerial->IsOpen())
    {
        pCommandControl->SensorSleep();
        m_ThreadFlag = 1;
        Sleep(100);
        pSerial->Close();

        SetDlgItemText(IDC_BTN_OPENPORT, L"开始采集");
    }
    else
    {
        pSerial->Open();
        char str[100];
        memset(str, '\0', 100);

        BOOL isExit = pCommandControl->QueryReceiverNodeVersion(str, 100);
        if(isExit)
        {
            m_Msg = "找到主节点存在";
            m_wndMessage.SetWindowText(m_Msg);
        }
        else
        {
            m_Msg = "主节点不存在";
            m_wndMessage.SetWindowText(m_Msg);
            return;
        }

        // 开启采集
        pCommandControl->StartCapture();

        // 进入数据模式
        pCommandControl->SensorToDataMode();

        // 线程开始读取
        m_ThreadFlag = 2;
        
        SetDlgItemText(IDC_BTN_OPENPORT, L"暂停采集");
    }
}


void CControlDlg::OnBnClickedBtnClossensor()
{
    m_ThreadFlag = 1;
    Sleep(100);
    pCommandControl->SensorPowerOff();
}


void CControlDlg::OnBnClickedBtnConnect()
{
	USES_CONVERSION;

    wchar_t strTmp[20];
    m_wndIP.GetWindowText(strTmp, 20);
	wchar_t strPort[10];
    m_wndPort.GetWindowText(strPort, 10);

    if (pPosTrackor->IsEnabled())
    {
        pPosTrackor->Disconnect();
    }
    pPosTrackor->Init();
	
	char* ip = W2A(strTmp);
	char* port = W2A(strPort);
	int nport = atoi(port);
	BOOL flag = pPosTrackor->ConnectTo(ip, nport);

    if (flag == false)
    {
        if (fpPos != NULL)
        {
            fclose(fpPos);
            fpPos = NULL;
        }
       MessageBox(L"连接光学设备失败");
       pPosTrackor->Disconnect();
       pPosTrackor->Release();
       return ;
    }
    m_wndMessage.SetWindowText(L"光学设备连接成功");
}


void CControlDlg::OnBnClickedBtnDisconnect()
{
    if (fpPos != NULL)
    {
        fclose(fpPos);
        fpPos =NULL;

        m_Msg = "pos记录文件已经关闭 位置：D:\\pos.txt";
        m_wndMessage.SetWindowText(m_Msg);
    }

    pPosTrackor->Disconnect();
    pPosTrackor->Release();
}

void CControlDlg::ReadDataFromSerial()
{
    int len = pSerial->Read(data, sizeof(data));
    PNPushData(data, len);
}

DWORD WINAPI CControlDlg::Acquisition(LPVOID param)
{
    CControlDlg* dlg = (CControlDlg*)param;

    if (dlg == NULL) return -1;

    while (true)
    {
        // 暂停
        if (dlg->m_ThreadFlag == 1)
        {
            Sleep(200);
            continue;
        }
        else if (dlg->m_ThreadFlag == 2)
        {
            // 从串口读取数据
            dlg->ReadDataFromSerial();
        }
        else
        {
            break;
        }
    }
    return 0;
}

void CControlDlg::OnBnClickedBtnWritefile()
{
    isWriteFile = (isWriteFile == false)?true:false;
    if (isWriteFile)
    {
        if (fpPos != NULL)
        {
            fclose(fpPos);
            fpPos = NULL;
        }

        fpPos = fopen("D:\\pos.txt", "w+");

        if (fpPos == NULL)
        {
            MessageBox(L"打开保存文件失败\n");
            return ;
        }
        char fileHeader[200];

        fprintf(fpPos, "Time   inert-X inert-Y inert-Z   optic-rX optic-rY optic-rZ   optic-X optic-Y optic-Z\n");

		SetDlgItemText(IDC_BTN_WRITEFILE, L"停止采集");
    }
    else
    {
        fclose(fpPos);
        fpPos =NULL;

        m_Msg = "pos记录文件已经关闭 位置：D:\\pos.txt\r\n  posCali数据保存在D：\\posCali.txt\r\n";
        m_wndMessage.SetWindowText(m_Msg);

		SetDlgItemText(IDC_BTN_WRITEFILE, L"光学采集");
    }
}

void CControlDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	isWriteFile = false;
	isWriteRawFile = false;
	// TODO: 在此添加专用代码和/或调用基类
	m_ThreadFlag = 0;
	Sleep(200);
	if (hThread != NULL)
	{
		CloseHandle(hThread);
		hThread = NULL;
	}

	if (pSerial != NULL)
	{
		delete pSerial;
	}
	if (pCommandControl != NULL)
	{
		delete pCommandControl;
	}
}


void CControlDlg::OnBnClickedBtnWriterawfile()
{
	isWriteRawFile = (isWriteRawFile == false) ? true : false;
	if (isWriteRawFile)
	{
		wnd_qType.EnableWindow(FALSE);

		if (fpInertia != NULL)
		{
			fclose(fpInertia);
			fpInertia = NULL;
		}

		fpInertia = fopen("D:\\inertia.txt", "w+");

		if (fpInertia == NULL)
		{
			MessageBox(L"打开保存文件失败\n");
			return;
		}

		CalibrationData data;
		PNGetCalibrationData(0, &data);
		fprintf(fpInertia, "Body Direction:%0.4f %0.4f %0.4f\n", data.FaceDirection.x, data.FaceDirection.y, data.FaceDirection.z);

		if (qType == QT_GlobalBoneQuat)
		{
			fprintf(fpInertia, "GlobalBoneQuat\n");
		}
		else if (qType == QT_GlobalRawQuat)
		{
			fprintf(fpInertia, "GlobalRawQuat\n");
		}

		fprintf(fpInertia, "Time hip-qs hip-qx hip-qy hip-qz hip-x hip-y hip-z head-qs head-qx head-qy head-qz head-x head-y head-z\n");
		
		if (fpOpt != NULL)
		{
			fclose(fpOpt);
			fpOpt = NULL;
		}

		fpOpt = fopen("D:\\Opt.txt", "w+");

		if (fpOpt == NULL)
		{
			MessageBox(L"打开保存文件失败\n");
			return;
		}

		fprintf(fpOpt, "Time Count nOtherMarks otherMark(x y z).........\n");

		SetDlgItemText(IDC_BTN_WRITERAWFILE, L"停止采集");
	}
	else
	{
		fclose(fpInertia);
		fpInertia = NULL;

		fclose(fpOpt);
		fpOpt = NULL;

		m_Msg = "记录文件已经保存\r\n 位置：D盘 文件名：inertia.txt Opt.txt";
		m_wndMessage.SetWindowText(m_Msg);
		wnd_qType.EnableWindow(TRUE);
		SetDlgItemText(IDC_BTN_WRITERAWFILE, L"开始采集原始数据");
	}
}


void CControlDlg::OnCbnSelchangeCombo1()
{
	int index = wnd_qType.GetCurSel();
	if (index == 0)
	{
		qType = QT_GlobalBoneQuat;
	}
	else if (index == 1)
	{
		qType = QT_GlobalRawQuat;
	}

	PNSetCalculatedQuaternionDataType(qType);
}


void CControlDlg::OnBnClickedButtonZero()
{
	// TODO:  在此添加控件通知处理程序代码
	PNZeroOutPosition(0);
}

#pragma once
#include "afxcmn.h"
#include "afxwin.h"

#include "Definitions.h"
#include "HybidTrack.h"

class PosTrackor;
class CSerialPort;
class SerialControl;



class CControlDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CControlDlg)

public:
	CControlDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CControlDlg();


// 对话框数据
	enum { IDD = IDD_DLG_CONTROL };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
    virtual BOOL OnInitDialog();
    virtual BOOL PreTranslateMessage(MSG* pMsg);
    CListCtrl m_wndRatioList;
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    CEdit m_wndIP;
    CEdit m_wndPort;
    afx_msg void OnBnClickedBtnTpos();
    afx_msg void OnBnClickedBtnApos();
    afx_msg void OnBnClickedBtnSpos();
    afx_msg void OnBnClickedBtnPos();
    afx_msg void OnBnClickedBtnOpenport();
    afx_msg void OnBnClickedBtnClossensor();
    afx_msg void OnBnClickedBtnConnect();
	afx_msg void OnBnClickedBtnDisconnect();
	afx_msg void OnBnClickedBtnWritefile();

	OutputQuaternionTypes qType;

    PosTrackor* pPosTrackor;
    FILE* fpPos;
	FILE* fpInertia;
	FILE* fpOpt;
   // FILE* fpCali;
    char tmpstr[200];
    CStatic m_wndProgress;

    // 光学实时头部位置数据
	Point3D_t OptiTracData;

	// OtherMark
	INT8 m_curOtherMarkN;
	float m_curOtherMark[3];

    // 
    CSerialPort* pSerial;
    SerialControl* pCommandControl;

    void ReadDataFromSerial();
    // 采集线程
    static  DWORD WINAPI Acquisition(LPVOID param); 
    HANDLE hThread;
    int m_ThreadFlag;

    // 是否写文件
    bool isWriteFile;
	bool isWriteRawFile;

    unsigned char data[1024];
    CEdit m_wndMsg;
    CString m_Msg;
    CStatic m_wndMessage;

	// 光学、惯性数据融合
    void DataFusion(float* bonePosX, Point3D_t optiTracData);
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedBtnWriterawfile();
	afx_msg void OnCbnSelchangeCombo1();
	CComboBox wnd_qType;
	afx_msg void OnBnClickedButtonZero();

	// 采集频率
	float DataFreq;

	// 构造一个混捕结构体，用于存储混合动捕的数据

	CHybidTrack HybidTrack;
	afx_msg void OnBnClickedBtnReadrawfile();
};

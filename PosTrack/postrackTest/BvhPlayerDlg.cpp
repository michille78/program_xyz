// BvhPlayerDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "postrack.h"
#include "BvhPlayerDlg.h"
#include "afxdialogex.h"

// CBvhPlayerDlg 对话框

IMPLEMENT_DYNAMIC(CBvhPlayerDlg, CDialogEx)

CBvhPlayerDlg::CBvhPlayerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CBvhPlayerDlg::IDD, pParent)
{

}

CBvhPlayerDlg::~CBvhPlayerDlg()
{
}

void CBvhPlayerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CBvhPlayerDlg, CDialogEx)
    ON_WM_SIZE()
END_MESSAGE_MAP()


// CBvhPlayerDlg 消息处理程序


BOOL CBvhPlayerDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // TODO:  在此添加额外的初始化
    PNCreateBvhPlayer(this->m_hWnd);

    PNCreateAvatar();

    // 加载上一次校准的文件
    PNLoadCalibrationData();

	PNSetSensorSuitType(SS_LegacySensors);

	PNSetSensorCombinationMode(0, SC_FullBody);

    // 设置一个带位移
    PNSetBvhDataFormat(TRUE, RO_YXZ);

    return TRUE;  // return TRUE unless you set the focus to a control
    // 异常: OCX 属性页应返回 FALSE
}


void CBvhPlayerDlg::OnSize(UINT nType, int cx, int cy)
{
    CDialogEx::OnSize(nType, cx, cy);
    PNBvhPlayerResizeToParent();
    // TODO: 在此处添加消息处理程序代码
}


BOOL CBvhPlayerDlg::PreTranslateMessage(MSG* pMsg)
{
    // TODO: 在此添加专用代码和/或调用基类
    if(pMsg->message==WM_KEYDOWN && pMsg->wParam==VK_ESCAPE)  return TRUE;
    if(pMsg->message==WM_KEYDOWN && pMsg->wParam==VK_RETURN) return TRUE; 
    return CDialogEx::PreTranslateMessage(pMsg);
}


BOOL CBvhPlayerDlg::DestroyWindow()
{
    // TODO: 在此添加专用代码和/或调用基类
    return CDialogEx::DestroyWindow();
}

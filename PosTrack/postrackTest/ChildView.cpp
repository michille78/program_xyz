
// ChildView.cpp : CChildView 类的实现
//

#include "stdafx.h"
#include "postrack.h"
#include "ChildView.h"
#include "BvhPlayerDlg.h"
#include "ControlDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CChildView

CChildView::CChildView()
{
}

CChildView::~CChildView()
{
    delete pControlDlg;
    delete pBvhPlayerDlg;
}


BEGIN_MESSAGE_MAP(CChildView, CWnd)
	ON_WM_PAINT()
    ON_WM_CREATE()
    ON_WM_SIZE()
END_MESSAGE_MAP()



// CChildView 消息处理程序

BOOL CChildView::PreCreateWindow(CREATESTRUCT& cs) 
{
	if (!CWnd::PreCreateWindow(cs))
		return FALSE;

	cs.dwExStyle |= WS_EX_CLIENTEDGE;
	cs.style &= ~WS_BORDER;
	cs.lpszClass = AfxRegisterWndClass(CS_HREDRAW|CS_VREDRAW|CS_DBLCLKS, 
		::LoadCursor(NULL, IDC_ARROW), reinterpret_cast<HBRUSH>(COLOR_WINDOW+1), NULL);

	return TRUE;
}

void CChildView::OnPaint() 
{
	CPaintDC dc(this); // 用于绘制的设备上下文
	
	// TODO: 在此处添加消息处理程序代码
	
	// 不要为绘制消息而调用 CWnd::OnPaint()
}



int CChildView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
    if (CWnd::OnCreate(lpCreateStruct) == -1)
        return -1;

    // TODO:  在此添加您专用的创建代码
    pControlDlg = new CControlDlg(this);
    pBvhPlayerDlg = new CBvhPlayerDlg(this);

    
    pBvhPlayerDlg->Create(IDD_DLG_BVHPLAYER, this);
    pBvhPlayerDlg->ShowWindow(SW_SHOW);

    pControlDlg->Create(IDD_DLG_CONTROL, this);
    pControlDlg->ShowWindow(SW_SHOW);

    return 0;
}


void CChildView::OnSize(UINT nType, int cx, int cy)
{
    CWnd::OnSize(nType, cx, cy);

    CRect rect;
    CChildView* pParent = ((CChildView*)GetParent());
    pParent->GetClientRect(&rect);

    CRect controlWndRect;
    pControlDlg->GetWindowRect(&controlWndRect);

    pBvhPlayerDlg->MoveWindow(rect.left, rect.top, rect.Width() - controlWndRect.Width(), rect.Height());
    pControlDlg->MoveWindow(rect.right - controlWndRect.Width(), rect.top, controlWndRect.Width(), rect.Height());
    // TODO: 在此处添加消息处理程序代码
}


BOOL CChildView::DestroyWindow()
{
    // TODO: 在此添加专用代码和/或调用基类


    return CWnd::DestroyWindow();
}


// ChildView.h : CChildView ��Ľӿ�
//


#pragma once

class CBvhPlayerDlg;
class CControlDlg;
// CChildView ����

class CChildView : public CWnd
{
// ����
public:
	CChildView();

// ����
public:
    CBvhPlayerDlg* pBvhPlayerDlg;
    CControlDlg* pControlDlg;
// ����
public:

// ��д
	protected:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// ʵ��
public:
	virtual ~CChildView();

	// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg void OnPaint();
	DECLARE_MESSAGE_MAP()
public:
    afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
    afx_msg void OnSize(UINT nType, int cx, int cy);
    virtual BOOL DestroyWindow();
};

#pragma once


// CBvhPlayerDlg 对话框

class CBvhPlayerDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CBvhPlayerDlg)

public:
	CBvhPlayerDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CBvhPlayerDlg();

// 对话框数据
	enum { IDD = IDD_DLG_BVHPLAYER };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
    virtual BOOL OnInitDialog();
    afx_msg void OnSize(UINT nType, int cx, int cy);
    virtual BOOL PreTranslateMessage(MSG* pMsg);
    virtual BOOL DestroyWindow();
};

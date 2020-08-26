/////////////////////////////////////////////////////////////////////////////
// Name:        keyboard.cpp
// Purpose:     Keyboard wxWidgets sample
// Author:      Vadim Zeitlin
// Modified by: Marcin Wojdyr
// Created:     07.04.02
// Copyright:   (c) 2002 Vadim Zeitlin
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

#include "wx/wxprec.h"
#include "wx/wx.h"

#include "uart.h"

#include <map>

//#ifdef __BORLANDC__
//    #pragma hdrstop
//#endif
//#ifndef WX_PRECOMP
//#endif

#include "icon.xpm"

void UartMessageDisplayCallback(const char *message) {
    fprintf(stdout, "%s", message);
}

// IDs for menu items
enum
{
    QuitID = wxID_EXIT,
    ClearID = wxID_CLEAR,
    SkipHook = 100,
    SkipDown,

    // These IDs must be in the same order as MyFrame::InputKind enum elements.
    IDInputCustom,
    IDInputEntry,
    IDInputText,

    TestAccelA,
    TestAccelCtrlA,
    TestAccelEsc,

    // Keyboard control frame
    KeyCtrlPad,
    UartConnect,
};

// Define a new frame type: this is going to be our main frame
class KeyPadFrame : public wxFrame
{
public:
    KeyPadFrame(const wxString& title, wxTextCtrl *p_logText);

private:
    // Set m_inputWin to either a new window of the given kind:
    enum InputKind
    {
        Input_Custom,   // Just a plain wxWindow
        Input_Entry,    // Single-line wxTextCtrl
        Input_Text      // Multi-line wxTextCtrl
    };

    void OnTestAccelA(wxCommandEvent& WXUNUSED(event))
        { m_logText->AppendText("Test accelerator \"A\" used.\n"); }
    void OnTestAccelCtrlA(wxCommandEvent& WXUNUSED(event))
        { m_logText->AppendText("Test accelerator \"Ctrl-A\" used.\n"); }
    void OnTestAccelEsc(wxCommandEvent& WXUNUSED(event))
        { m_logText->AppendText("Test accelerator \"Esc\" used.\n"); }

    void OnSkipDown(wxCommandEvent& event) { m_skipDown = event.IsChecked(); }
    void OnSkipHook(wxCommandEvent& event) { m_skipHook = event.IsChecked(); }

    void OnUartConnect(wxCommandEvent& event);

    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);
    void OnChar(wxKeyEvent& event) { LogEvent("Char", event); event.Skip(); }
    void OnCharHook(wxKeyEvent& event)
    {
        // The logged messages can be confusing if the input window doesn't
        // have focus so warn about this.
        if ( !m_inputWin->HasFocus() )
        {
            m_logText->SetDefaultStyle(*wxRED);
            m_logText->AppendText("WARNING: focus is not on input window, "
                                  "non-hook events won't be logged.\n");
            m_logText->SetDefaultStyle(wxTextAttr());
        }

        LogEvent("Hook", event);
        if ( m_skipHook )
            event.Skip();
    }

    void LogEvent(const wxString& name, wxKeyEvent& event);

    void OnInputWindowKind(wxCommandEvent& event);

    // event handlers
    void DoCreateInputWindow(InputKind inputKind);
    void OnPaintInputWin(wxPaintEvent& event);

    wxWindow   *m_inputWin;
    wxTextCtrl *m_logText;

    bool m_skipHook,
         m_skipDown;

    enum class KeyState{
        KEY_PUSHED,
        KEY_RELEASED
    };
    std::map<int, KeyState> key_state;
};

// Define a new frame type: this is going to be our main frame
class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString& title);

private:
    // event handlers
    void OnQuit(wxCommandEvent& WXUNUSED(event)) { 
        if (m_keypad_frame) {
            delete m_keypad_frame;
        }
        Close(true);
    }
    void OnAbout(wxCommandEvent& event);

    void OnClear(wxCommandEvent& WXUNUSED(event)) { m_logText->Clear(); }
    void OnKeyControlPad(wxCommandEvent& WXUNUSED(event));

    void OnUartConnect(wxCommandEvent& event);

    wxTextCtrl *m_logText;
    KeyPadFrame *m_keypad_frame;

    UartDriverLite  *m_uart;
};

// Define a new application type, each program should derive a class from wxApp
class MyApp : public wxApp
{
public:
    // 'Main program' equivalent: the program execution "starts" here
    virtual bool OnInit()
    {
        // create the main application window
        new MyFrame("Keyboard wxWidgets App");

        // If we returned false here, the application would exit immediately.
        return true;
    }
};

// Create a new application object: this macro will allow wxWidgets to create
// the application object during program execution (it's better than using a
// static object for many reasons) and also declares the accessor function
// wxGetApp() which will return the reference of the right type (i.e. MyApp and
// not wxApp)
IMPLEMENT_APP(MyApp)


// ============================================================================
// implementation
// ============================================================================

// frame constructor
MyFrame::MyFrame(const wxString& title)
       : wxFrame(NULL, wxID_ANY, title),
        m_keypad_frame(NULL)
{
    SetIcon(wxICON(sample));

    // create a menu bar
    wxMenu *menuFile = new wxMenu;

    // the "File" menu list
    menuFile->Append(ClearID, "&Clear log\tCtrl-L");
    menuFile->AppendSeparator();

    menuFile->Append(KeyCtrlPad, "&Key Pad\tK",
        "Pop-up key pad pannel to control tank.");
    menuFile->Append(UartConnect, "&Uart Connect\tU",
        "Connects UART.");
    menuFile->AppendSeparator();


    menuFile->Append(QuitID, "E&xit\tAlt-X", "Quit this program");

    // the "Help" menu list
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT, "&About\tF1", "Show about dialog");


    // now append the freshly created menu to the menu bar...
    wxMenuBar *menuBar = new wxMenuBar();
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    // ... and attach this menu bar to the frame
    SetMenuBar(menuBar);


    wxTextCtrl *headerText = new wxTextCtrl(this, wxID_ANY, "",
                                            wxDefaultPosition, wxDefaultSize,
                                            wxTE_READONLY);
    headerText->SetValue(
               " event          key     KeyCode mod   UnicodeKey  "
               "  RawKeyCode RawKeyFlags  Position");
    
    // Log window
    m_logText = new wxTextCtrl(this, wxID_ANY, "",
                               wxDefaultPosition, wxDefaultSize,
                               wxTE_MULTILINE|wxTE_READONLY|wxTE_RICH|wxHSCROLL);

    // set monospace font to have output in nice columns
    wxFont font(10, wxFONTFAMILY_TELETYPE,
                wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    headerText->SetFont(font);
    m_logText->SetFont(font);


    // layout
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    //sizer->Add(m_inputWin, wxSizerFlags().Expand());
    sizer->Add(headerText, wxSizerFlags().Expand());
    sizer->Add(m_logText, wxSizerFlags(1).Expand());
    SetSizerAndFit(sizer);

    // set size and position on screen
    SetSize(700, 340);
    CentreOnScreen();


    // connect menu event handlers
    Connect(QuitID, wxEVT_MENU,
            wxCommandEventHandler(MyFrame::OnQuit));

    Connect(wxID_ABOUT, wxEVT_MENU,
            wxCommandEventHandler(MyFrame::OnAbout));

    Connect(ClearID, wxEVT_MENU,
            wxCommandEventHandler(MyFrame::OnClear));

    Connect(UartConnect, wxEVT_MENU,
            wxCommandEventHandler(MyFrame::OnUartConnect));

    // Event handler connection.
    Connect(KeyCtrlPad, wxEVT_MENU,
            wxCommandEventHandler(MyFrame::OnKeyControlPad));

    // status bar is useful for showing the menu items help strings
    CreateStatusBar();

    // and show itself (the frames, unlike simple controls, are not shown when
    // created initially)
    Show(true);
}

// event handlers
void MyFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    wxMessageBox("Demonstrates keyboard event processing in wxWidgets\n"
                 "(c) 2002 Vadim Zeitlin\n"
                 "(c) 2008 Marcin Wojdyr",
                 "About wxWidgets Keyboard Sample",
                 wxOK | wxICON_INFORMATION, this);
}

void MyFrame::OnKeyControlPad(wxCommandEvent& WXUNUSED(event))
{
    m_keypad_frame = new KeyPadFrame("Control pannel", m_logText);
}

void MyFrame::OnUartConnect(wxCommandEvent& event)
{
    m_uart = new UartDriverLite("/dev/ttyUSB0");

    if (m_uart->OpenChannelUart() < 0) {
        m_logText->AppendText("[ERROR] UART connect is failed.\n");
    }
    else {
        m_logText->AppendText("[INFO] UART connect is success.\n");
    }
}

// helper function that returns textual description of wx virtual keycode
const char* GetVirtualKeyCodeName(int keycode)
{
    switch ( keycode )
    {
#define WXK_(x) \
        case WXK_##x: return #x;

        WXK_(BACK)
        WXK_(TAB)
        WXK_(RETURN)
        WXK_(ESCAPE)
        WXK_(SPACE)
        WXK_(DELETE)
        WXK_(START)
        WXK_(LBUTTON)
        WXK_(RBUTTON)
        WXK_(CANCEL)
        WXK_(MBUTTON)
        WXK_(CLEAR)
        WXK_(SHIFT)
        WXK_(ALT)
        WXK_(CONTROL)
        WXK_(MENU)
        WXK_(PAUSE)
        WXK_(CAPITAL)
        WXK_(END)
        WXK_(HOME)
        WXK_(LEFT)
        WXK_(UP)
        WXK_(RIGHT)
        WXK_(DOWN)
        WXK_(SELECT)
        WXK_(PRINT)
        WXK_(EXECUTE)
        WXK_(SNAPSHOT)
        WXK_(INSERT)
        WXK_(HELP)
        WXK_(NUMPAD0)
        WXK_(NUMPAD1)
        WXK_(NUMPAD2)
        WXK_(NUMPAD3)
        WXK_(NUMPAD4)
        WXK_(NUMPAD5)
        WXK_(NUMPAD6)
        WXK_(NUMPAD7)
        WXK_(NUMPAD8)
        WXK_(NUMPAD9)
        WXK_(MULTIPLY)
        WXK_(ADD)
        WXK_(SEPARATOR)
        WXK_(SUBTRACT)
        WXK_(DECIMAL)
        WXK_(DIVIDE)
        WXK_(F1)
        WXK_(F2)
        WXK_(F3)
        WXK_(F4)
        WXK_(F5)
        WXK_(F6)
        WXK_(F7)
        WXK_(F8)
        WXK_(F9)
        WXK_(F10)
        WXK_(F11)
        WXK_(F12)
        WXK_(F13)
        WXK_(F14)
        WXK_(F15)
        WXK_(F16)
        WXK_(F17)
        WXK_(F18)
        WXK_(F19)
        WXK_(F20)
        WXK_(F21)
        WXK_(F22)
        WXK_(F23)
        WXK_(F24)
        WXK_(NUMLOCK)
        WXK_(SCROLL)
        WXK_(PAGEUP)
        WXK_(PAGEDOWN)
        WXK_(NUMPAD_SPACE)
        WXK_(NUMPAD_TAB)
        WXK_(NUMPAD_ENTER)
        WXK_(NUMPAD_F1)
        WXK_(NUMPAD_F2)
        WXK_(NUMPAD_F3)
        WXK_(NUMPAD_F4)
        WXK_(NUMPAD_HOME)
        WXK_(NUMPAD_LEFT)
        WXK_(NUMPAD_UP)
        WXK_(NUMPAD_RIGHT)
        WXK_(NUMPAD_DOWN)
        WXK_(NUMPAD_PAGEUP)
        WXK_(NUMPAD_PAGEDOWN)
        WXK_(NUMPAD_END)
        WXK_(NUMPAD_BEGIN)
        WXK_(NUMPAD_INSERT)
        WXK_(NUMPAD_DELETE)
        WXK_(NUMPAD_EQUAL)
        WXK_(NUMPAD_MULTIPLY)
        WXK_(NUMPAD_ADD)
        WXK_(NUMPAD_SEPARATOR)
        WXK_(NUMPAD_SUBTRACT)
        WXK_(NUMPAD_DECIMAL)
        WXK_(NUMPAD_DIVIDE)

        WXK_(WINDOWS_LEFT)
        WXK_(WINDOWS_RIGHT)
#ifdef __WXOSX__
        WXK_(RAW_CONTROL)
#endif
#undef WXK_

    default:
        return NULL;
    }
}

// helper function that returns textual description of key in the event
wxString GetKeyName(const wxKeyEvent &event)
{
    int keycode = event.GetKeyCode();
    const char* virt = GetVirtualKeyCodeName(keycode);
    if ( virt )
        return virt;
    if ( keycode > 0 && keycode < 32 )
        return wxString::Format("Ctrl-%c", (unsigned char)('A' + keycode - 1));
    if ( keycode >= 32 && keycode < 128 )
        return wxString::Format("'%c'", (unsigned char)keycode);

#if wxUSE_UNICODE
    int uc = event.GetUnicodeKey();
    if ( uc != WXK_NONE )
        return wxString::Format("'%c'", uc);
#endif

    return "unknown";
}


// Key-pad frame constructor
KeyPadFrame::KeyPadFrame(const wxString& title, wxTextCtrl *p_logText)
       : wxFrame(NULL, wxID_ANY, title),
         m_inputWin(NULL),
         m_skipHook(true),
         m_skipDown(true)
{

    m_logText = p_logText;
    m_logText->AppendText("Keyboard control pannel is opened.\n");

    // create a menu bar
    wxMenu *menuFile = new wxMenu;

    menuFile->Append(TestAccelA, "Test accelerator &1\tA");
    menuFile->Append(TestAccelCtrlA, "Test accelerator &2\tCtrl-A");
    menuFile->Append(TestAccelEsc, "Test accelerator &3\tEsc");
    menuFile->AppendSeparator();

    menuFile->AppendCheckItem(SkipHook, "Skip CHAR_HOOK event",
        "Not skipping this event disables both KEY_DOWN and CHAR events"
    );
    menuFile->Check(SkipHook, true);
    menuFile->AppendCheckItem(SkipDown, "Skip KEY_DOWN event",
        "Not skipping this event disables CHAR event generation"
    );
    menuFile->Check(SkipDown, true);
    menuFile->AppendSeparator();

    menuFile->AppendRadioItem(IDInputCustom, "Use &custom control\tCtrl-C",
        "Use custom wxWindow for input window"
    );
    menuFile->AppendRadioItem(IDInputEntry, "Use text &entry\tCtrl-E",
        "Use single-line wxTextCtrl for input window"
    );
    menuFile->AppendRadioItem(IDInputText, "Use &text control\tCtrl-T",
        "Use multi-line wxTextCtrl for input window"
    );
    menuFile->AppendSeparator();

    DoCreateInputWindow(Input_Custom);

    Connect(SkipHook, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnSkipHook));
    Connect(SkipDown, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnSkipDown));

    Connect(IDInputCustom, IDInputText, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnInputWindowKind));

    Connect(TestAccelA, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnTestAccelA));

    Connect(TestAccelCtrlA, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnTestAccelCtrlA));

    Connect(TestAccelEsc, wxEVT_MENU,
            wxCommandEventHandler(KeyPadFrame::OnTestAccelEsc));

    // notice that we don't connect OnCharHook() to the input window, unlike
    // the usual key events this one is propagated upwards
    //Connect(wxEVT_CHAR_HOOK, wxKeyEventHandler(KeyPadFrame::OnCharHook));


    // now append the freshly created menu to the menu bar...
    wxMenuBar *menuBar = new wxMenuBar();
    menuBar->Append(menuFile, "&File");
 
    // ... and attach this menu bar to the frame
    SetMenuBar(menuBar);


    // layout
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(m_inputWin, wxSizerFlags(1).Expand());
    SetSizerAndFit(sizer);

    // set size and position on screen
    SetSize(300, 200);
    //CentreOnScreen();

    Show(true);
}

void KeyPadFrame::DoCreateInputWindow(InputKind inputKind)
{
    wxWindow* const oldWin = m_inputWin;

    switch ( inputKind )
    {
        case Input_Custom:
            m_inputWin = new wxWindow(this, wxID_ANY,
                                      wxDefaultPosition, wxSize(-1, 50),
                                      wxRAISED_BORDER);
            break;

        case Input_Entry:
            m_inputWin = new wxTextCtrl(this, wxID_ANY, "Press keys here");
            break;

        case Input_Text:
            m_inputWin = new wxTextCtrl(this, wxID_ANY, "Press keys here",
                                        wxDefaultPosition, wxSize(-1, 50),
                                        wxTE_MULTILINE);
            break;
    }

    m_inputWin->SetBackgroundColour(*wxBLUE);
    m_inputWin->SetForegroundColour(*wxWHITE);

    // connect event handlers for the blue input window
    m_inputWin->Connect(wxEVT_KEY_DOWN, wxKeyEventHandler(KeyPadFrame::OnKeyDown),
                        NULL, this);
    m_inputWin->Connect(wxEVT_KEY_UP, wxKeyEventHandler(KeyPadFrame::OnKeyUp),
                        NULL, this);
    //m_inputWin->Connect(wxEVT_CHAR, wxKeyEventHandler(KeyPadFrame::OnChar),
    //                    NULL, this);

    if ( inputKind == Input_Custom )
    {
        m_inputWin->Connect(wxEVT_PAINT,
                            wxPaintEventHandler(KeyPadFrame::OnPaintInputWin),
                            NULL, this);
    }

    if ( oldWin )
    {
        GetSizer()->Replace(oldWin, m_inputWin);
        Layout();
        delete oldWin;
    }
}

void KeyPadFrame::OnInputWindowKind(wxCommandEvent& event)
{
    DoCreateInputWindow(
        static_cast<InputKind>(event.GetId() - IDInputCustom)
    );
}

void KeyPadFrame::OnPaintInputWin(wxPaintEvent& WXUNUSED(event))
{
    wxPaintDC dc(m_inputWin);
    dc.SetTextForeground(*wxWHITE);
    wxFont font(*wxSWISS_FONT);
    font.SetWeight(wxFONTWEIGHT_BOLD);
    font.SetPointSize(font.GetPointSize() + 2);
    dc.SetFont(font);

    dc.DrawLabel("Press keys here",
                 m_inputWin->GetClientRect(), wxALIGN_CENTER);
}

void KeyPadFrame::LogEvent(const wxString& name, wxKeyEvent& event)
{
    wxString msg;
    // event  key_name  KeyCode  modifiers  Unicode  raw_code raw_flags pos
    msg.Printf("%7s %15s %5d   %c%c%c%c"
#if wxUSE_UNICODE
                   "%5d (U+%04x)"
#else
                   "    none   "
#endif
#ifdef wxHAS_RAW_KEY_CODES
                   "  %7lu    0x%08lx"
#else
                   "  not-set    not-set"
#endif
                   "  (%5d,%5d)"
                   "\n",
               name,
               GetKeyName(event),
               event.GetKeyCode(),
               event.ControlDown() ? 'C' : '-',
               event.AltDown()     ? 'A' : '-',
               event.ShiftDown()   ? 'S' : '-',
               event.MetaDown()    ? 'M' : '-'
#if wxUSE_UNICODE
               , event.GetUnicodeKey()
               , event.GetUnicodeKey()
#endif
#ifdef wxHAS_RAW_KEY_CODES
               , (unsigned long) event.GetRawKeyCode()
               , (unsigned long) event.GetRawKeyFlags()
#endif
               , event.GetX()
               , event.GetY()
               );

    m_logText->AppendText(msg);
}

void KeyPadFrame::OnKeyDown(wxKeyEvent& event)
{
    int keycode = event.GetKeyCode();
    if (key_state.find(keycode) == key_state.end() || 
        key_state[keycode] == KeyState::KEY_RELEASED) {
        LogEvent("KeyDown", event);
        key_state[keycode] = KeyState::KEY_PUSHED;
    }

    if ( m_skipDown )
        event.Skip();
}

void KeyPadFrame::OnKeyUp(wxKeyEvent& event) {
    // LogEvent("KeyUp", event);
    int keycode = event.GetKeyCode();
    if (key_state.find(keycode) == key_state.end() || 
        key_state[keycode] == KeyState::KEY_PUSHED) {
        LogEvent("KeyUp", event);
        key_state[keycode] = KeyState::KEY_RELEASED;
    }
}


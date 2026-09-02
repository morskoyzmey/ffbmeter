// ============================================================================
//  FFB Meter — измерение отклика DirectDrive рулевой базы
// ----------------------------------------------------------------------------

//  MIT License
//
//  Copyright(c) 2026 morskoyzmey
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files(the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions :
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

//  Метод: короткий импульс Constant Force заданной длительности (мс) и силы (%)
//  после центрирования. Серия измерений в разном направлении.
//  Позиция оси опрашивается busy-wait'ом (до ~4 кГц) с QueryPerformanceCounter.
//  Метрики:
//    * Задержка срабатывания — после вызова Start() до первого движения 
//     (THREAD_PRIORITY_TIME_CRITICAL, busy-wait,  t0 = конец вызова Start, 
//      подтверждение движения относительно шумовой полки)
//    * Средняя скорость (75%-100%) — (pulseX - percentX)/(t_pulse - t_percent), град/с и об/мин
//    * Среднее ускорение           — avgV/(t_vmax - t_start), град/с² и об/мин/с
//    * Амплитуда                   — град
//  После каждого импульса руль возвращается в центр P-D регулятором
//  (краевые фильтры баз не влияют на следующее измерение).
//  Вывод — выровненная читаемая таблица; кнопка «Копировать» кладёт в буфер
//  обмена TSV-версию данных (вставка в Google Sheets по колонкам).
//
//  Сборка (MSVC) через Developer PowerShell for VS 2022:
//    rc app.rc (один раз при смене иконки)
// 
//    cl /EHsc /O2 /utf-8 /DUNICODE /D_UNICODE ffb_meter.cpp app.res
// ============================================================================

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#define DIRECTINPUT_VERSION 0x0800
#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <windowsx.h>
#include <dinput.h>
#include <mmsystem.h>
#include <vector>
#include <string>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cwchar>
#include <algorithm>
#include "resource.h"
using std::max;
using std::min;

#ifdef _MSC_VER
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")

#endif

// ---------------------------------------------------------------- IDs -------
enum {
    IDC_COMBO_DEV = 1001,
    IDC_BTN_REFRESH,
    IDC_EDIT_PULSE,
    IDC_EDIT_FORCE,
    IDC_EDIT_REPEATS,
    IDC_EDIT_PAUSE,
    IDC_EDIT_WHEELDEG,
    IDC_EDIT_THRESH,
    IDC_CHK_ALTDIR,
    IDC_BTN_START,
    IDC_BTN_STOP,
    IDC_BTN_COPY,
    IDC_BTN_CLEAR,
    IDC_EDIT_OUT,
    IDC_STATIC_STATUS
};

#define WM_APP_ADDROW  (WM_APP + 1)   // lParam = wchar_t* (heap) — строка в окно
#define WM_APP_STATUS  (WM_APP + 2)   // lParam = wchar_t* (heap) — статус
#define WM_APP_DONE    (WM_APP + 3)
#define WM_APP_ADDTSV  (WM_APP + 4)   // lParam = wchar_t* (heap) — строка в TSV-буфер

static int g_wheelPolarity = 0; // 0 = ещё не определена в этой сессии

const int tailMs = 5;
const float g_avgSpeedPercent = 0.75;
// ------------------------------------------------------------- globals ------
struct DevInfo { GUID guid; std::wstring name; };

static HINSTANCE               g_hInst;
static HWND                    g_hMain, g_hOut, g_hStatus, g_hCombo;
static HFONT                   g_hFontUI, g_hFontMono;
static IDirectInput8*          g_pDI = nullptr;
static std::vector<DevInfo>    g_devices;
static std::atomic<bool>       g_running(false);
static std::atomic<bool>       g_stopRequest(false);
static HANDLE                  g_hThread = nullptr;
static std::wstring            g_tsv;      // данные для кнопки «Копировать» (TSV)

struct MeasureParams {
    GUID   devGuid;
    int    pulseMs;
    int    forcePct;
    int    repeats;
    int    pauseMs;
    double wheelDeg;
    double threshDeg;
    bool   altDir;
    HWND   hNotify;
};

// ==== Графики ====================================================
#define WM_APP_ADDGRAPH   (WM_APP + 10)   // lParam = GraphData* (владение переходит UI)
#define WM_APP_CLEARGRAPH (WM_APP + 11)   // очистка перед новой серией

#define IDC_BTN_PREV  201
#define IDC_BTN_NEXT  202

struct GraphPt { float t; float deg; };   // t, мс от Start(); угол, градусы (отн. базовой линии)

struct GraphData {
    std::vector<GraphPt> pts;
    std::vector<GraphPt> raw_pts;
    float pulseMs;   // длительность импульса
    float latMs;     // измеренная задержка, -1 если не определена
    float thrDeg;    // порог срабатывания, градусы
    float ltDeg;     // <-- порог линейной зоны symlog, считается один раз при добавлении
    int   idx;       // номер импульса (1..N)
    float forcePct;  // сила, % (для подписи)
    float acc;       // ускорение на интервале порога срабатывания до длины импульса
    float avgV;      // средняя скорость на интервале порога срабатывания до длины импульса   
    float noise;     // уровень шума в градусах
};

static std::vector<GraphData*> g_graphs;  // владеет UI-поток
static int  g_curGraph = -1;
static bool  g_hasMouse = false;
static int g_mouseX = 0;
static int   g_snapIdx = -1;   // индекс в raw_pts текущего графика, к которому примагничен крест
static HWND g_hGraph = nullptr, g_hGraphLbl = nullptr, g_hCopyright = nullptr;
// «Красивый» шаг сетки: 1/2/5 * 10^k
static double NiceStep(double range, int targetDivs)
{
    if (range <= 0) return 1.0;
    double raw = range / targetDivs;
    double mag = pow(10.0, floor(log10(raw)));
    double n = raw / mag;
    double s = (n < 1.5) ? 1.0 : (n < 3.5) ? 2.0 : (n < 7.5) ? 5.0 : 10.0;
    return s * mag;
}

static double LogFwd(double v, double floorV)
{
    double av = (v < 0.0) ? -v : v; // на всякий случай, хотя данные уже >= 0
    if (av < floorV) av = floorV;
    return log10(av);
}

static void SetCurGraph(int i);

static void ClearGraphsList()
{
    for (auto* g : g_graphs) delete g;
    g_graphs.clear();
    g_curGraph = -1;
    g_snapIdx = -1;          // <-- см. ниже, индекс примагниченной точки тоже сбрасываем
    SetCurGraph(-1);
}

static double ComputeLinThreshold(const GraphData* gd)
{
    double limitT = (gd->latMs >= 0) ? gd->latMs : (double)gd->pulseMs;

    double minNonZero = HUGE_VAL;
    for (const auto& p : gd->pts) {
        if (p.t > limitT) break;
        if (p.deg > 1e-6 && p.deg < minNonZero) minNonZero = p.deg;
    }
    if (minNonZero == HUGE_VAL)
        minNonZero = (double)gd->thrDeg * 0.05;

    const double eps = 0.001; // ° — минимально осмысленный "пол" шкалы
    return max(minNonZero * 0.5, eps);
}

struct GraphGeom { RECT pl; double xMin, xMax; };

static GraphGeom ComputeGraphGeom(const RECT& rc, const GraphData* gd)
{
    GraphGeom g{};
    const int ML = 52, MR = 14, MT = 12, MB = 26;
    g.pl = { rc.left + ML, rc.top + MT, rc.right - MR, rc.bottom - MB };
    g.xMin = 0.0;
    g.xMax = gd ? (gd->pulseMs + tailMs) : 1.0;
    return g;
}

static int ScreenXForT(const GraphGeom& g, double t)
{
    double span = g.xMax - g.xMin;
    if (span <= 0) return g.pl.left;
    return g.pl.left + (int)((t - g.xMin) / span * (g.pl.right - g.pl.left) + 0.5);
}

static int FindNearestRawIndexByPixelX(const GraphData* gd, const GraphGeom& geom, int px)
{
    if (!gd || gd->raw_pts.empty()) return -1;

    double tTarget = geom.xMin + (double)(px - geom.pl.left) /
        (geom.pl.right - geom.pl.left) * (geom.xMax - geom.xMin);

    const auto& pts = gd->raw_pts;
    // raw_pts заполняются последовательно по возрастанию t в MeasureThread — можно бинарным поиском
    auto it = std::lower_bound(pts.begin(), pts.end(), (float)tTarget,
        [](const GraphPt& p, float val) { return p.t < val; });

    if (it == pts.begin()) return 0;
    if (it == pts.end())   return (int)pts.size() - 1;

    size_t i = it - pts.begin();
    double dRight = fabs((double)pts[i].t - tTarget);
    double dLeft = fabs((double)pts[i - 1].t - tTarget);
    return (int)((dLeft <= dRight) ? (i - 1) : i);
}

static void DrawGraph(HDC dc, const RECT& rc, const GraphData* gd)
{
    // Фон
    HBRUSH bg = CreateSolidBrush(RGB(24, 24, 28));
    FillRect(dc, &rc, bg);
    DeleteObject(bg);

    HFONT font = CreateFontW(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");

    HFONT oldFont = (HFONT)SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);

    if (!gd || gd->pts.empty()) {
        SetTextColor(dc, RGB(140, 140, 140));
        RECT r = rc;
        DrawTextW(dc, L"Нет данных — запустите измерение", -1, &r,
            DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        SelectObject(dc, oldFont); DeleteObject(font);
        return;
    }

    // Область построения
    GraphGeom geom = ComputeGraphGeom(rc, gd);
    RECT pl = geom.pl;
    if (pl.right - pl.left < 40 || pl.bottom - pl.top < 40) {
        SelectObject(dc, oldFont); DeleteObject(font);
        return;
    }

    HFONT font2 = CreateFontW(-18, 0, 0, 0, FW_BOLD, 0, 0, 0,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");

    // Диапазоны
    // Максимум по модулю (данные уже неотрицательные)
    double maxAbs = (double)gd->thrDeg;
    for (const auto& p : gd->pts)
        if (p.deg > maxAbs) maxAbs = p.deg;

    const double lt = gd->ltDeg;           // "пол" шкалы (минимально отображаемое значение)

    double uMin = log10(lt);
    double uMax = log10(max(maxAbs, lt * 1.001));
    double padU = (uMax - uMin) * 0.08;
    if (padU < 1e-6) padU = 0.2;
    uMax += padU;
    // uMin не паддим вниз — это "дно" шкалы, ниже физически нет смысла

    auto Y = [&](double d) -> int {
        double u = LogFwd(d, lt);
        return pl.bottom - (int)((u - uMin) / (uMax - uMin) * (pl.bottom - pl.top) + 0.5);
        };

    // Порог должен быть виден
    const double xMin = geom.xMin, xMax = geom.xMax;
    auto X = [&](double t) -> int { return ScreenXForT(geom, t); };

    // Сетка + подписи
    HPEN penGrid = CreatePen(PS_SOLID, 1, RGB(48, 48, 56));
    HPEN old = (HPEN)SelectObject(dc, penGrid);
    SetTextColor(dc, RGB(150, 150, 158));
    wchar_t buf[64];

    double xs = 1;// NiceStep(xMax - xMin, 8);
    for (double t = 0; t <= xMax + 1e-6; t += xs) {
        int x = X(t);
        MoveToEx(dc, x, pl.top, nullptr); LineTo(dc, x, pl.bottom);
        swprintf(buf, 64, L"%g", t);
        RECT tr = { x - 30, pl.bottom + 2, x + 30, pl.bottom + 20 };
        DrawTextW(dc, buf, -1, &tr, DT_CENTER | DT_TOP | DT_SINGLELINE);
    }

    std::vector<double> ticks;
    const double thr = (int((gd->thrDeg) * 100)) * 0.01;// Приводим к сетке

    // Вверх от порога: thr, 2·thr, 5·thr, 10·thr, 20·thr, 50·thr, ...
    for (double mult = 1.0; ; ) {
        double v = thr * mult;
        if (v > maxAbs * 1.0001) break;
        ticks.push_back(v);
        if (mult == 1.0) mult = 2.0;
        else if (mult == 2.0) mult = 5.0;
        else { mult *= 2.0; } // после 5 переходим в 10,20,50,100...
    }

    // Вниз от порога: thr/2, thr/5, thr/10, thr/20, thr/50, ...
    for (double div = 2.0; ; ) {
        double v = thr / div;
        if (v < lt * 0.9999) break;
        ticks.push_back(v);
        if (div == 2.0) div = 5.0;
        else if (div == 5.0) div = 10.0;
        else { div *= 2.0; } // после 10 -> 20,50,100...
    }

    if (ticks.empty()) ticks.push_back(thr);

    for (double v : ticks) {
        int y = Y(v);
        if (y < pl.top - 1 || y > pl.bottom + 1) continue;
        MoveToEx(dc, pl.left, y, nullptr); LineTo(dc, pl.right, y);
        swprintf(buf, 64, L"%.3g\u00B0", v);
        RECT tr = { rc.left, y - 8, pl.left - 4, y + 8 };
        DrawTextW(dc, buf, -1, &tr, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }

    // "Дно" шкалы — условная линия у пола лога (визуальная база вместо нуля)
    HPEN penZero = CreatePen(PS_SOLID, 1, RGB(90, 90, 100));
    SelectObject(dc, penZero);
    MoveToEx(dc, pl.left, pl.bottom, nullptr); LineTo(dc, pl.right, pl.bottom);


    // Уровень шума
    {
        HPEN penThr = CreatePen(PS_DOT, 1, RGB(230, 80, 80));
        SelectObject(dc, penThr);
        MoveToEx(dc, pl.left, Y(gd->noise), nullptr); LineTo(dc, X(gd->latMs), Y(gd->noise));
        DeleteObject(penThr);

        SelectObject(dc, font);
        SetTextColor(dc, RGB(230, 80, 80));
        swprintf(buf, 64, L"Ур. шума: %.3f\u00B0", gd->noise);
        TextOutW(dc, pl.left + 4, Y(gd->noise) - 20, buf, (int)wcslen(buf));
    }

    // Порог thr (пунктир, только положительная сторона)
    {
        HPEN penThr = CreatePen(PS_DOT, 1, RGB(255, 170, 60));
        SelectObject(dc, penThr);
        MoveToEx(dc, pl.left, Y(gd->thrDeg), nullptr); LineTo(dc, X(gd->latMs), Y(gd->thrDeg));
        DeleteObject(penThr);

        SelectObject(dc, font);
        SetTextColor(dc, RGB(255, 170, 60));
        swprintf(buf, 64, L"Порог: %.3f\u00B0", gd->thrDeg);
        TextOutW(dc, pl.left + 4, Y(gd->thrDeg) - 20, buf, (int)wcslen(buf));
    }        

    // Конец импульса (вертикаль)
    HPEN penPulse = CreatePen(PS_DOT, 1, RGB(110, 110, 190));
    SelectObject(dc, penPulse);
    MoveToEx(dc, X(gd->pulseMs), pl.top, nullptr); LineTo(dc, X(gd->pulseMs), pl.bottom);
    
    {
        SelectObject(dc, font2);

        SetTextColor(dc, RGB(110, 110, 190));
        swprintf(buf, 64, L"%.0f мс", gd->pulseMs);
        TextOutW(dc, X(gd->pulseMs) + 4, pl.top + 2, buf, (int)wcslen(buf));
    }

    // Маркер задержки (вертикаль, красный)
    if (gd->latMs >= 0)
    {
        SelectObject(dc, font2);

        HPEN penLat = CreatePen(PS_SOLID, 1, RGB(230, 80, 80));
        SelectObject(dc, penLat);
        int x = X(gd->latMs);
        MoveToEx(dc, x, pl.top, nullptr); LineTo(dc, x, pl.bottom);
        SetTextColor(dc, RGB(230, 80, 80));
        swprintf(buf, 64, L"%.2f мс", gd->latMs);
        TextOutW(dc, x + 4, pl.top + 2, buf, (int)wcslen(buf));
        DeleteObject(penLat); 
    }

    // --- Маркеры точек замера (raw) ---
    if (gd->raw_pts.size())
    {
        std::vector<POINT> poly;
        poly.reserve(gd->raw_pts.size());
        for (const auto& p : gd->raw_pts)
            poly.push_back({ X(p.t), Y(p.deg) });

        HBRUSH brMark = CreateSolidBrush(RGB(0, 0, 0));
        HPEN   penMarkBorder = CreatePen(PS_SOLID, 1, RGB(255, 170, 60));
        HGDIOBJ oldBr = SelectObject(dc, brMark);
        HGDIOBJ oldPn = SelectObject(dc, penMarkBorder);

        const int R = 3;              // радиус маркера в пикселях
        const int MIN_PX_GAP = 4;     // прореживание: не ближе N px друг к другу

        POINT last = { -10000, -10000 };
        for (size_t i = 0; i < poly.size(); ++i) {
            const POINT& pt = poly[i];
            bool isEdge = (i == 0 || i == poly.size() - 1);
            long dx = pt.x - last.x, dy = pt.y - last.y;
            if (!isEdge && (dx * dx + dy * dy) < (MIN_PX_GAP * MIN_PX_GAP))
                continue;
            Ellipse(dc, pt.x - R, pt.y - R, pt.x + R + 1, pt.y + R + 1);
            last = pt;
        }

        SelectObject(dc, oldBr);
        SelectObject(dc, oldPn);
        DeleteObject(brMark);
        DeleteObject(penMarkBorder);
    }
    // --- Маркеры точек замера (сглаженные) ---
    if (gd->pts.size())
    {
        std::vector<POINT> poly;
        poly.reserve(gd->pts.size());
        for (const auto& p : gd->pts)
            poly.push_back({ X(p.t), Y(p.deg) });

       
       HPEN penCurve = CreatePen(PS_SOLID, 1, RGB(80, 200, 120));
        SelectObject(dc, penCurve);
       if (poly.size() >= 2)
            Polyline(dc, poly.data(), (int)poly.size());

        DeleteObject(penCurve);


        HBRUSH brMark = CreateSolidBrush(RGB(80, 200, 120));
        HPEN   penMarkBorder = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
        HGDIOBJ oldBr = SelectObject(dc, brMark);
        HGDIOBJ oldPn = SelectObject(dc, penMarkBorder);

        const int R = 3;              // радиус маркера в пикселях
        const int MIN_PX_GAP = 4;     // прореживание: не ближе N px друг к другу

        POINT last = { -10000, -10000 };
        for (size_t i = 0; i < poly.size(); ++i) {
            const POINT& pt = poly[i];
            bool isEdge = (i == 0 || i == poly.size() - 1);
            long dx = pt.x - last.x, dy = pt.y - last.y;
            if (!isEdge && (dx * dx + dy * dy) < (MIN_PX_GAP * MIN_PX_GAP))
                continue;
            Ellipse(dc, pt.x - R, pt.y - R, pt.x + R + 1, pt.y + R + 1);
            last = pt;
        }

        SelectObject(dc, oldBr);
        SelectObject(dc, oldPn);
        DeleteObject(brMark);
        DeleteObject(penMarkBorder);
    }


    // Кривая ускорения
    {
        SelectObject(dc, font2);
        HPEN penLat = CreatePen(PS_SOLID, 1, RGB(80, 200, 120));
        SelectObject(dc, penLat);
 
        float impulseEndValue = 0;
        float midValue = 0;
        float valuePercent = 0;
        float impulseMid = gd->latMs + (gd->pulseMs - gd->latMs) * 0.5;
        float impulsePercent = gd->latMs + (gd->pulseMs - gd->latMs) * g_avgSpeedPercent;
        float impulsePercentRawMS = 0;

        for (const auto& p : gd->pts)
        {
            if (p.t <= impulseMid) midValue = p.deg;
            if (p.t <= impulsePercent)
            {
                valuePercent = p.deg;
                impulsePercentRawMS = p.t;
            }

            if (p.t <= gd->pulseMs) impulseEndValue = p.deg;
            else
            {
               // impulseEndValue = p.deg;
                break;
            }
        }
        impulsePercent = impulsePercentRawMS;

        int startX = X(gd->latMs);
        int endX = X(gd->pulseMs);
        int endY = Y(impulseEndValue);
        int percX = X(impulsePercent);
        int percY = Y(valuePercent);

        SetTextColor(dc, RGB(255, 255, 255));
        swprintf(buf, 64, L"%.0f об/м/с", gd->acc);
        TextOutW(dc, X(impulseMid) + 5, Y(midValue) + 5, buf, (int)wcslen(buf));

        swprintf(buf, 64, L"%.0f об/м", gd->avgV);
        TextOutW(dc, percX + 5, endY - 25, buf, (int)wcslen(buf));

        DeleteObject(penLat);


        SetTextColor(dc, RGB(80, 200, 120));

        SelectObject(dc, font);
        //swprintf(buf, 64, L"%.2f\u00B0", valuePercent);
        //TextOutW(dc, percX - 65, percY - 15, buf, (int)wcslen(buf));

        swprintf(buf, 64, L"%.0f%%", g_avgSpeedPercent*100.0);
        TextOutW(dc, percX + 5, pl.top, buf, (int)wcslen(buf));

        swprintf(buf, 64, L"%.2f\u00B0", impulseEndValue);
        TextOutW(dc, pl.right - 50 + 5, endY - 7, buf, (int)wcslen(buf));

        {
            HPEN penThr = CreatePen(PS_DOT, 1, RGB(80, 200, 120));
            SelectObject(dc, penThr);

            MoveToEx(dc, percX, endY, nullptr); LineTo(dc, pl.right - 50, endY);
            DeleteObject(penThr);
        }
        {
            HPEN penThr = CreatePen(PS_SOLID, 1, RGB(80, 200, 120));
            SelectObject(dc, penThr);

            MoveToEx(dc, percX, percY, nullptr); LineTo(dc, percX, pl.top);
            DeleteObject(penThr);
        }

        // Горизонтальная пунктирная - замер ускорения
        HPEN penThr = CreatePen(PS_DOT, 1, RGB(230, 80, 80));
        SelectObject(dc, penThr);
        MoveToEx(dc, startX, Y(midValue), nullptr); LineTo(dc, endX, Y(midValue));
        DeleteObject(penThr);
        
    }


    // Рамка
    HPEN penFrame = CreatePen(PS_SOLID, 1, RGB(70, 70, 80));
    SelectObject(dc, penFrame);
    MoveToEx(dc, pl.left, pl.top, nullptr);
    LineTo(dc, pl.right, pl.top); LineTo(dc, pl.right, pl.bottom);
    LineTo(dc, pl.left, pl.bottom); LineTo(dc, pl.left, pl.top);

    SelectObject(dc, old);

    // --- Перекрестие, примагниченное к ближайшей точке замера ---
    if (g_hasMouse && g_snapIdx >= 0 && g_snapIdx < (int)gd->raw_pts.size())
    {
        SelectObject(dc, font2);

        const GraphPt& sp = gd->raw_pts[g_snapIdx];
        int mx = X(sp.t);
        int my = Y(sp.deg);

        HPEN penCross = CreatePen(PS_DOT, 1, RGB(220, 220, 220));
        HGDIOBJ oldPenC = SelectObject(dc, penCross);
        MoveToEx(dc, pl.left, my, nullptr); LineTo(dc, pl.right, my);
        MoveToEx(dc, mx, pl.top, nullptr); LineTo(dc, mx, pl.bottom);
        SelectObject(dc, oldPenC);
        DeleteObject(penCross);

        HBRUSH brSnap = CreateSolidBrush(RGB(255, 210, 60));
        HPEN   penSnapB = CreatePen(PS_SOLID, 1, RGB(120, 90, 0));
        HGDIOBJ oldBr = SelectObject(dc, brSnap);
        HGDIOBJ oldPn = SelectObject(dc, penSnapB);
        Ellipse(dc, mx - 4, my - 4, mx + 5, my + 5);
        SelectObject(dc, oldBr);
        SelectObject(dc, oldPn);
        DeleteObject(brSnap);
        DeleteObject(penSnapB);

        wchar_t lbl[64];
        swprintf(lbl, 64, L" t=%.2f мс   |\u0394|=%.3f\u00B0 ", sp.t, sp.deg);

        SIZE sz;
        GetTextExtentPoint32W(dc, lbl, (int)wcslen(lbl), &sz);
        int lx = mx + 10;
        int ly = my + 10;//my - sz.cy - 8;
        if (lx + sz.cx + 6 > pl.right)
        {
            lx = mx - sz.cx - 10;
            ly = my - sz.cy - 8;
        }
        //if (ly < pl.top)               ly = my + 10;
        if (ly > pl.bottom - sz.cy - 8) ly = my - sz.cy - 8;

        RECT bgr = { lx - 3, ly - 2, lx + sz.cx + 3, ly + sz.cy + 2 };
        HBRUSH brBg = CreateSolidBrush(RGB(30, 30, 35));
        FillRect(dc, &bgr, brBg);
        DeleteObject(brBg);
        FrameRect(dc, &bgr, (HBRUSH)GetStockObject(GRAY_BRUSH));

        int oldBk = SetBkMode(dc, TRANSPARENT);
        COLORREF oldClr = SetTextColor(dc, RGB(255, 255, 255));
        TextOutW(dc, lx, ly, lbl, (int)wcslen(lbl));
        SetTextColor(dc, oldClr);
        SetBkMode(dc, oldBk);
    }

    DeleteObject(penGrid); 
    DeleteObject(penZero); 
    DeleteObject(penPulse); 
    DeleteObject(penFrame);
    DeleteObject(font2);
    DeleteObject(font);

    SelectObject(dc, oldFont); 
}


static void SetCurGraph(int i); // fwd

static LRESULT CALLBACK GraphWndProc(HWND h, UINT m, WPARAM w, LPARAM l)
{
    switch (m) {
    case WM_ERASEBKGND:
        return 1; // всё рисуем сами, без мерцания

    case WM_MOUSEMOVE: {
        if (g_curGraph < 0 || g_curGraph >= (int)g_graphs.size()) return 0;
        const GraphData* gd = g_graphs[g_curGraph];

        int mx = GET_X_LPARAM(l);
        int my = GET_Y_LPARAM(l);

        bool firstEnter = !g_hasMouse;
        g_hasMouse = true;

        if (firstEnter) {
            TRACKMOUSEEVENT tme{ sizeof(tme) };
            tme.dwFlags = TME_LEAVE;
            tme.hwndTrack = h;
            TrackMouseEvent(&tme);
        }

        RECT rc; GetClientRect(h, &rc);
        GraphGeom geom = ComputeGraphGeom(rc, gd);

        if (mx < geom.pl.left || mx > geom.pl.right ||
            my < geom.pl.top || my > geom.pl.bottom)
        {
            if (g_snapIdx != -1) { g_snapIdx = -1; InvalidateRect(h, nullptr, FALSE); }
            return 0;
        }

        g_mouseX = mx;

        int idx = FindNearestRawIndexByPixelX(gd, geom, mx);
        if (idx == g_snapIdx) return 0;   // точка та же — перерисовка не нужна

        g_snapIdx = idx;
        InvalidateRect(h, nullptr, FALSE);
        return 0;
    }
    case WM_MOUSELEAVE: {
        if (g_hasMouse || g_snapIdx != -1) {
            g_hasMouse = false;
            g_snapIdx = -1;
            InvalidateRect(h, nullptr, FALSE);
        }
        return 0;
    }
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(h, &ps);
        RECT rc; GetClientRect(h, &rc);
        // двойная буферизация
        HDC mem = CreateCompatibleDC(dc);
        HBITMAP bmp = CreateCompatibleBitmap(dc, rc.right, rc.bottom);
        HBITMAP oldBmp = (HBITMAP)SelectObject(mem, bmp);
        const GraphData* gd =
            (g_curGraph >= 0 && g_curGraph < (int)g_graphs.size())
            ? g_graphs[g_curGraph] : nullptr;

        if (g_hasMouse)
        {
            RECT rc; GetClientRect(h, &rc);
            GraphGeom geom = ComputeGraphGeom(rc, gd);

            int idx = FindNearestRawIndexByPixelX(gd, geom, g_mouseX);
            g_snapIdx = idx;
        }

        DrawGraph(mem, rc, gd);
        BitBlt(dc, 0, 0, rc.right, rc.bottom, mem, 0, 0, SRCCOPY);
        SelectObject(mem, oldBmp);
        DeleteObject(bmp); DeleteDC(mem);
        EndPaint(h, &ps);
        return 0;
    }
    case WM_LBUTTONDOWN:
        SetFocus(h);
        return 0;
    case WM_KEYDOWN:
        if (w == VK_LEFT)  SetCurGraph(g_curGraph - 1);
        if (w == VK_RIGHT) SetCurGraph(g_curGraph + 1);
        return 0;
    case WM_MOUSEWHEEL:
        SetCurGraph(g_curGraph + (GET_WHEEL_DELTA_WPARAM(w) > 0 ? -1 : 1));
        return 0;
    case WM_SIZE:
        InvalidateRect(h, nullptr, FALSE);
        return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

static void SetCurGraph(int i)
{
    g_snapIdx = -1;   // <-- индекс точки относится к конкретному графику

    if (g_graphs.empty()) { g_curGraph = -1; }
    else {
        if (i < 0) i = 0;
        if (i >= (int)g_graphs.size()) i = (int)g_graphs.size() - 1;
        g_curGraph = i;
    }
    wchar_t buf[96];
    if (g_curGraph >= 0) {
        const GraphData* gd = g_graphs[g_curGraph];
        swprintf(buf, 96, L"Импульс %d/%d  (сила %.0f%%, %.0f мс)",
            g_curGraph + 1, (int)g_graphs.size(), gd->forcePct, gd->pulseMs);
    }
    else {
        wcscpy_s(buf, L"Нет графиков");
    }
    SetWindowTextW(g_hGraphLbl, buf);
    InvalidateRect(g_hGraph, nullptr, FALSE);
}

static void RegisterGraphClass(HINSTANCE hInst)
{
    WNDCLASSW wc = {};
    wc.lpfnWndProc = GraphWndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"FfbLatGraphWnd";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr; // рисуем сами (WM_ERASEBKGND -> 1)
    RegisterClassW(&wc);
}

// ------------------------------------------------------------ helpers -------
static void PostText(HWND h, UINT msg, const wchar_t* s) {
    PostMessageW(h, msg, 0, (LPARAM)_wcsdup(s));
}
static void SetStatus(HWND h, const wchar_t* s) { PostText(h, WM_APP_STATUS, s); }

static int GetEditInt(HWND parent, int id, int def) {
    wchar_t buf[64] = L"";
    GetWindowTextW(GetDlgItem(parent, id), buf, 63);
    int v = _wtoi(buf);
    return (v != 0 || buf[0] == L'0') ? v : def;
}
static float GetEditFloat(HWND parent, int id) {
    wchar_t buf[64] = L"";
    GetWindowTextW(GetDlgItem(parent, id), buf, 63);
    return _wtof(buf);
}
// ------------------------------------------------ DirectInput enumeration ---
static BOOL CALLBACK EnumFFDevCb(const DIDEVICEINSTANCEW* inst, VOID*) {
    DevInfo d;
    d.guid = inst->guidInstance;
    d.name = inst->tszProductName;
    g_devices.push_back(d);
    return DIENUM_CONTINUE;
}

static void RefreshDevices() {
    g_devices.clear();
    g_wheelPolarity = 0;
    SendMessageW(g_hCombo, CB_RESETCONTENT, 0, 0);
    if (!g_pDI) return;
    g_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumFFDevCb, nullptr,
                       DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK);
    for (auto& d : g_devices)
        SendMessageW(g_hCombo, CB_ADDSTRING, 0, (LPARAM)d.name.c_str());
    if (!g_devices.empty())
        SendMessageW(g_hCombo, CB_SETCURSEL, 0, 0);
    wchar_t st[128];
    swprintf(st, 128, L"Найдено FFB-устройств: %d", (int)g_devices.size());
    SetWindowTextW(g_hStatus, st);
}

// -------------------------------------------------------- measurement -------
struct Sample { double t; LONG x; };

static bool PollX(IDirectInputDevice8* dev, LONG& x) {
    DIJOYSTATE2 js;
    HRESULT hr = dev->Poll();
    if (FAILED(hr)) { dev->Acquire(); dev->Poll(); }
    hr = dev->GetDeviceState(sizeof(js), &js);
    if (hr == DIERR_INPUTLOST || hr == DIERR_NOTACQUIRED) {
        dev->Acquire();
        hr = dev->GetDeviceState(sizeof(js), &js);
    }
    if (FAILED(hr)) return false;
    x = js.lX;
    return true;
}


// Определяет знак связи "команда силы -> направление движения руля".
// Короткими пробными импульсами возрастающей амплитуды толкает руль
// и смотрит, в какую сторону реально сдвинулось положение.
// Возвращает +1 / -1. Если движения не обнаружено вообще (FFB не работает,
// руль заблокирован) — возвращает +1 как безопасный дефолт.
static int DetectPolarity(IDirectInputDevice8* dev, IDirectInputEffect* eff,
    DIEFFECT& ef, DICONSTANTFORCE& cf,
    double unitsPerDeg, double qpcToMs)
{
    const LONG   probeForces[] = { 200, 350, 500 }; // возрастающая амплитуда
    const double probeMs = 120.0;             // длительность импульса
    const double minMoveDeg = 0.5;               // порог надёжного обнаружения

    for (LONG mag : probeForces) 
    {
        for (int sgn : { +1, -1 }) 
        {
            LONG x0;
            if (!PollX(dev, x0)) continue;

            LONG force = sgn * mag;
            cf.lMagnitude = force;
            eff->SetParameters(&ef, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);

            LARGE_INTEGER t0, tc; QueryPerformanceCounter(&t0);
            LONG xr = x0;
            double dtMs = 0;
            while (dtMs < probeMs) {
                if (!PollX(dev, xr)) { Sleep(1); }
                QueryPerformanceCounter(&tc);
                dtMs = (tc.QuadPart - t0.QuadPart) * qpcToMs;
            }

            cf.lMagnitude = 0;
            eff->SetParameters(&ef, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
            Sleep(30); // дать погаситься колебаниям перед следующей попыткой/оценкой

            double moveDeg = (xr - x0) / unitsPerDeg;
            if (fabs(moveDeg) >= minMoveDeg) {
                int appliedSign = (force > 0) ? 1 : -1;
                int moveSign = (moveDeg > 0) ? 1 : -1;
                return appliedSign * moveSign;
            }
            // недостаточно сдвинулись — пробуем другой знак/большую амплитуду
            if (g_stopRequest) break;
        }

        if (g_stopRequest)
        {
            cf.lMagnitude = 0;
            eff->SetParameters(&ef, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
            eff->Stop();
            break;
        }
    }
    return 1; // не удалось определить — считаем совпадающей полярность
}

// ------------------------------------------------- возврат руля в центр -----
// P-D регулятор на Constant Force: гонит руль к центру диапазона, чтобы
// каждое измерение стартовало из одной точки (вдали от краевых фильтров).
// g_wheelPolarity — знак связи «сила -> движение»
static void CenterWheel(IDirectInputDevice8* dev, IDirectInputEffect* eff,
                        DIEFFECT& ef, DICONSTANTFORCE& cf,
                        double centerX, double unitsPerDeg,
                        double qpcToMs)
{
    cf.lMagnitude = 0;
    ef.dwDuration = INFINITE;
    if (FAILED(eff->SetParameters(&ef, DIEP_DURATION | DIEP_TYPESPECIFICPARAMS)))
        return;
    if (FAILED(eff->Start(1, 0))) return;

    if (g_wheelPolarity == 0)
        g_wheelPolarity = DetectPolarity(dev, eff, ef, cf, unitsPerDeg, qpcToMs);
    const int polarity = g_wheelPolarity;

    const double Kp   = 100.0;   // ед. силы на градус ошибки
    const double Kd   = 10.0;    // ед. силы на град/с — демпфирование
    const LONG   Fmax = 1000;    // лимит силы 10% — безопасно для DD

    LARGE_INTEGER t0, tc; QueryPerformanceCounter(&t0);
    double prevX = 0, prevT = -1, inTolSince = -1;

    for (;;) {
        QueryPerformanceCounter(&tc);
        double t = (tc.QuadPart - t0.QuadPart) * qpcToMs;
        if (t > 3000.0) break;                       // таймаут 3 с

        LONG xr;
        if (!PollX(dev, xr)) { Sleep(2); continue; }

        double errDeg  = (centerX - xr) / unitsPerDeg;
        double velDegS = 0;
        if (prevT >= 0 && t > prevT)
            velDegS = ((xr - prevX) / unitsPerDeg) / ((t - prevT) / 1000.0);
        prevX = xr; prevT = t;

        LONG f = (LONG)(polarity * (Kp * errDeg - Kd * velDegS));
        if (f >  Fmax) f =  Fmax;
        if (f < -Fmax) f = -Fmax;
        cf.lMagnitude = f;
        eff->SetParameters(&ef, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);

        // стоп: |ошибка| < 0.7° и |скорость| < 8 °/с непрерывно 300 мс
        if (fabs(errDeg) < 0.1 && fabs(velDegS) < 0.25) {
            if (inTolSince < 0) inTolSince = t;
            else if (t - inTolSince > 300.0) break;
        } else inTolSince = -1;
        Sleep(2);

        if (g_stopRequest) break;
    }
    cf.lMagnitude = 0;
    eff->SetParameters(&ef, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
    eff->Stop();
}

static DWORD WINAPI MeasureThread(LPVOID pv) {
    MeasureParams P = *(MeasureParams*)pv;
    delete (MeasureParams*)pv;
    HWND hN = P.hNotify;

    timeBeginPeriod(1);

    g_wheelPolarity = 0;

    IDirectInputDevice8* dev = nullptr;
    IDirectInputEffect*  eff = nullptr;
    HRESULT hr;
    LARGE_INTEGER qpf; QueryPerformanceFrequency(&qpf);
    const double qpcToMs = 1000.0 / (double)qpf.QuadPart;

    do {
        hr = g_pDI->CreateDevice(P.devGuid, &dev, nullptr);
        if (FAILED(hr)) { SetStatus(hN, L"Ошибка: CreateDevice"); break; }
        dev->SetDataFormat(&c_dfDIJoystick2);
        hr = dev->SetCooperativeLevel(g_hMain, DISCL_EXCLUSIVE | DISCL_BACKGROUND);
        if (FAILED(hr)) { SetStatus(hN, L"Ошибка: SetCooperativeLevel (EXCLUSIVE)"); break; }

        DIPROPDWORD dw = {};
        dw.diph.dwSize = sizeof(DIPROPDWORD);
        dw.diph.dwHeaderSize = sizeof(DIPROPHEADER);
        dw.diph.dwHow = DIPH_DEVICE;
        dw.dwData = DIPROPAUTOCENTER_OFF;
        dev->SetProperty(DIPROP_AUTOCENTER, &dw.diph);

        DIPROPRANGE rg = {};
        rg.diph.dwSize = sizeof(DIPROPRANGE);
        rg.diph.dwHeaderSize = sizeof(DIPROPHEADER);
        rg.diph.dwHow = DIPH_BYOFFSET;
        rg.diph.dwObj = DIJOFS_X;
        rg.lMin = -32768; rg.lMax = 32767;
        dev->SetProperty(DIPROP_RANGE, &rg.diph);
        dev->GetProperty(DIPROP_RANGE, &rg.diph);
        const double span = (double)rg.lMax - (double)rg.lMin;
        const double unitsPerDeg = span / (P.wheelDeg > 1 ? P.wheelDeg : 900.0);
        const double centerX = ((double)rg.lMin + (double)rg.lMax) / 2.0;

        hr = dev->Acquire();
        if (FAILED(hr)) { SetStatus(hN, L"Ошибка: Acquire"); break; }

        DWORD axes[1] = { DIJOFS_X };
        LONG  dir[1]  = { 1 };
        DICONSTANTFORCE cf = {};
        DIEFFECT ef = {};
        ef.dwSize                = sizeof(DIEFFECT);
        ef.dwFlags               = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
        ef.dwDuration            = (DWORD)P.pulseMs * 1000;
        ef.dwSamplePeriod        = 0;
        ef.dwGain                = DI_FFNOMINALMAX;
        ef.dwTriggerButton       = DIEB_NOTRIGGER;
        ef.cAxes                 = 1;
        ef.rgdwAxes              = axes;
        ef.rglDirection          = dir;
        ef.lpEnvelope            = nullptr;
        ef.cbTypeSpecificParams  = sizeof(DICONSTANTFORCE);
        ef.lpvTypeSpecificParams = &cf;
        ef.dwStartDelay          = 0;
        cf.lMagnitude = DI_FFNOMINALMAX * P.forcePct / 100;

        hr = dev->CreateEffect(GUID_ConstantForce, &ef, &eff, nullptr);
        if (FAILED(hr) || !eff) { SetStatus(hN, L"Ошибка: CreateEffect (ConstantForce)"); break; }

        // --- заголовки: читаемые в окно + TSV в буфер копирования ----------
        PostText(hN, WM_APP_ADDROW,
            L"  №  ±   Задерж   Порог   Ср.ускор   Ср.ускор    Ср.скор    Угол");
        PostText(hN, WM_APP_ADDROW,
            L"             мс    град    град/с²     об/м/с     об/мин    град");
        PostText(hN, WM_APP_ADDROW,
            L"--- --- -------  ------  ---------  ---------  ---------  ------");
        PostText(hN, WM_APP_ADDTSV,
            L"№\tНапр\tЗадержка,мс\tПорог,град\t"
            L"Ср.ускор,град/с²\tСр.ускор,об/мин/с\t"
            L"Ср.скор,об/мин\t"
            L"Угол,град");

        std::vector<Sample> smp; smp.reserve(8192);
        double sumLat = 0, sumAngle = 0, sumAccAv = 0, sumVel = 0, sumThrDeg = 0;
        int okCount = 0;

        PostMessage(hN, WM_APP_CLEARGRAPH, 0, 0);


        for (int i = 1; i <= P.repeats && !g_stopRequest; ++i) {

            SetStatus(hN, L"Возврат руля в центр...");
            CenterWheel(dev, eff, ef, cf, centerX, unitsPerDeg, qpcToMs);

            for (int w = 0; w < P.pauseMs && !g_stopRequest; w += 10) Sleep(10);

            if (g_stopRequest) break;

            wchar_t st[128];
            swprintf(st, 128, L"Импульс %d из %d ...", i, P.repeats);
            SetStatus(hN, st);

            // --- базовая линия: 300 мс тишины -----------------------------
            double x0 = 0, noise = 0;
            {
                std::vector<LONG> base;
                LARGE_INTEGER tS, tC; QueryPerformanceCounter(&tS);
                for (;;) {
                    LONG x;
                    if (PollX(dev, x)) base.push_back(x);
                    QueryPerformanceCounter(&tC);
                    if ((tC.QuadPart - tS.QuadPart) * qpcToMs >= 300.0) break;
                    Sleep(1);
                }
                if (base.empty()) { SetStatus(hN, L"Ошибка чтения оси"); break; }
                double s = 0; for (LONG v : base) s += v;
                x0 = s / base.size();
                for (LONG v : base) noise = max(noise, fabs(v - x0));
            }
            double thr = max(noise * 1.5, P.threshDeg * unitsPerDeg);

            int sign = (P.altDir && (i % 2 == 0)) ? -1 : 1;
            cf.lMagnitude = sign * DI_FFNOMINALMAX * P.forcePct / 100;
            ef.dwDuration = (DWORD)P.pulseMs * 1000;
            eff->SetParameters(&ef, DIEP_DURATION | DIEP_TYPESPECIFICPARAMS);

            // --- старт эффекта + запись (анти-джиттер) ---------------------
            smp.clear();
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
            LARGE_INTEGER tb, ta, tc;
            QueryPerformanceCounter(&tb);
            hr = eff->Start(1, 0);
            QueryPerformanceCounter(&ta);
            LONGLONG t0q = ta.QuadPart;// tb.QuadPart + (ta.QuadPart - tb.QuadPart) / 2;// Замер времени ровно с середины вызова Start() - а зачем?
            if (FAILED(hr)) {
                SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
                SetStatus(hN, L"Ошибка: Start эффекта"); break;
            }

            const double recMs = P.pulseMs + tailMs;
            double lastT = -1.0;
            for (;;) {
                QueryPerformanceCounter(&tc);
                double t = (tc.QuadPart - t0q) * qpcToMs;
                if (t > recMs) break;
                if (t - lastT >= 0.25) {              // до 4 кГц
                    LONG x;
                    if (PollX(dev, x)) { smp.push_back({ t, x }); lastT = t; }
                }
                YieldProcessor();                     // busy-wait без Sleep
            }
            eff->Stop();
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);

            // --- метрики ---------------------------------------------------
            if (smp.size() < 20) continue;
            int n = (int)smp.size();

            // задержка: интерполяция пересечения порога + подтверждение
            double lat = -1;
            for (int k = 1; k < n; ++k) {
                double dPrev = fabs(smp[k - 1].x - x0);
                double dCur  = fabs(smp[k].x - x0);
                if (dCur > thr && dPrev <= thr) {
                    int kc = min(k + 3, n - 1);
                    if (fabs(smp[kc].x - x0) < thr * 0.8) continue;  // выброс
                    lat = (dCur > dPrev)
                        ? smp[k - 1].t + (thr - dPrev) * (smp[k].t - smp[k - 1].t) / (dCur - dPrev)
                        : smp[k].t;
                    break;
                }
            }

            std::vector<double> xs(n);                // сглаживание, окно 11
            for (int k = 0; k < n; ++k) {
                double s = 0; int c = 0;
                for (int j = k - 5; j <= k + 5; ++j)
                    if (j >= 0 && j < n) { s += fabs(smp[j].x - x0); ++c; }// Отрицательные выбросы по модулю
                xs[k] = s / c ;
            }

            // скорость / ускорение (центральные разности), окно импульс
            double avgV = 0, t_pulse = -1;
            double percentX = 0;
            double t_percent = 0;
            double midDeg = 0;
            double pulseX = 0;
            double lastDeg = 0;
            for (int k = 2; k < n - 2; ++k) {
                if (smp[k].t < lat) continue;// Не считаем скорости до порога

                if (smp[k].t > P.pulseMs) break;

                if (smp[k].t <= lat + (P.pulseMs - lat) * g_avgSpeedPercent)
                {
                    percentX = xs[k];// берем значение угла со сглаженного графика
                    t_percent = smp[k].t;
                }

                t_pulse = smp[k].t;
                pulseX = xs[k];// fabs(smp[k].x - x0);
            }
            lastDeg = pulseX / unitsPerDeg;
            midDeg = percentX / unitsPerDeg;

            // среднее ускорение разгона: от начала движения до пика скорости
            double aavg = 0;

            if (lat >= 0 && t_pulse > lat)
            {
                double avg_v_dt = t_pulse - t_percent;
                avgV = (pulseX - percentX) / avg_v_dt; // ед/мс

                double pulse_dt = (t_pulse - lat);

                aavg = avgV / pulse_dt;  // ед/мс²
            }

            // пересчёт: сырые -> градусы -> RPM (град/с / 6)
            double velDegS  = avgV * 1000.0 / unitsPerDeg; // град/с
            double velRPM   = velDegS / 6.0;// об/мин
 
            double aavgDegS = aavg * 1.0e6 / unitsPerDeg; // град/с2
            double aavgRPMs = aavgDegS / 6.0;// об/мин/с
            
            wchar_t row[320], tsv[320];
            const wchar_t* dirS = sign > 0 ? L"+" : L"-";
            if (lat < 0) {
                swprintf(row, 320, L"%3d %2ls  %7ls %6ls %9ls  %9ls  %9ls   %8ls   %4ls",
                         i, dirS, L"нет движ", L"-", L"-", L"-", L"-", L"-", L"-");
                swprintf(tsv, 320, L"%d\t%ls\tнет движ.\t-\t-\t-\t-\t-\t-",
                         i, dirS);
            } else {
                swprintf(row, 320, L"%3d %2ls  %7.2f  %6.3f  %9.0f  %9.0f   %8.1f   %5.2f",
                         i, dirS, lat, thr / unitsPerDeg, aavgDegS, aavgRPMs,velRPM, lastDeg);
                swprintf(tsv, 320, L"%d\t%ls\t%.2f\t%.3f\t%.0f\t%.0f\t%.1f\t%.2f",
                         i, dirS, lat, thr / unitsPerDeg,  aavgDegS, aavgRPMs, velRPM, lastDeg);
                sumAngle += lastDeg;
                sumLat += lat; sumAccAv += aavgDegS;
                sumVel += velDegS;
                sumThrDeg += thr / unitsPerDeg;
                ++okCount;
            }
            PostText(hN, WM_APP_ADDROW, row);
            PostText(hN, WM_APP_ADDTSV, tsv);

            {
                GraphData* gd = new GraphData;
                gd->noise = noise / unitsPerDeg;// град
                gd->acc = aavgRPMs;// об/мин/с
                gd->avgV = avgV * 1000.0 / unitsPerDeg / 6.0;// об/мин
                gd->pulseMs = (float)P.pulseMs;
                gd->latMs = (float)lat;
                gd->thrDeg = (float)(thr / unitsPerDeg);      // твой коэффициент raw -> градусы
                gd->idx = i + 1;
                gd->forcePct = (float)P.forcePct;
                const double tEnd = P.pulseMs + tailMs;            // окно: импульс + tailMs
                gd->pts.reserve(smp.size());
                int index = 0;
                for (const auto& s : smp) {
                    double t = s.t; // как в расчёте задержки
                    if (t < 0.0 || t > tEnd) continue;
                    gd->pts.push_back({ (float)t, (float)( xs[index] / unitsPerDeg )});
                    gd->raw_pts.push_back({ (float)t, (float)( fabs(s.x - x0) / unitsPerDeg ) });
                    ++index;
                }
                PostMessage(hN, WM_APP_ADDGRAPH, 0, (LPARAM)gd);
            }
        }

        // --- строка средних значений --------------------------------------
        if (okCount > 0) {
            double mLat = sumLat / okCount;
            double mAav = sumAccAv / okCount, mVel = sumVel / okCount;
            double mAngle = sumAngle / okCount;
            double mThrDeg = sumThrDeg / okCount;
            wchar_t row[320], tsv[320];
            PostText(hN, WM_APP_ADDROW,
                L"--- --- -------  ------  ---------  ---------  ---------  ------");
            swprintf(row, 320, L"%3ls %3ls %7.2f  %6.3f  %9.0f  %9.0f   %8.1f   %5.2f",
                     L"Ср.", L"", mLat, mThrDeg, mAav, mAav / 6.0, mVel / 6.0, mAngle);
            swprintf(tsv, 320, L"Среднее\t\t%.2f\t%.3f\t%.0f\t%.0f\t%.1f\t%.2f",
                     mLat, mThrDeg, mAav, mAav / 6.0, mVel / 6.0, mAngle);
            PostText(hN, WM_APP_ADDROW, row);
            PostText(hN, WM_APP_ADDTSV, tsv);
        }

        if (!g_stopRequest)
        {
            SetStatus(hN, L"Возврат руля в центр...");
            CenterWheel(dev, eff, ef, cf, centerX, unitsPerDeg, qpcToMs);
        }

        SetStatus(hN, g_stopRequest ? L"Остановлено." : L"Готово.");

    } while (false);

    if (eff) { eff->Stop(); eff->Release(); }
    if (dev) { dev->Unacquire(); dev->Release(); }
    timeEndPeriod(1);
    PostMessageW(hN, WM_APP_DONE, 0, 0);
    return 0;
}

// ================================================================ UI =========

static HWND MkLabel(HWND p, const wchar_t* txt, int x, int y, int w) {
    HWND h = CreateWindowW(L"STATIC", txt, WS_CHILD | WS_VISIBLE,
                           x, y + 3, w, 18, p, nullptr, g_hInst, nullptr);
    SendMessageW(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    return h;
}
static HWND MkEdit(HWND p, int id, const wchar_t* txt, int x, int y, int w) {
    HWND h = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", txt,
                             WS_CHILD | WS_VISIBLE | ES_NUMBER | ES_RIGHT,
                             x, y, w, 22, p, (HMENU)(INT_PTR)id, g_hInst, nullptr);
    SendMessageW(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    return h;
}
static HWND MkBtn(HWND p, int id, const wchar_t* txt, int x, int y, int w, int hgt = 26) {
    HWND h = CreateWindowW(L"BUTTON", txt, WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                           x, y, w, hgt, p, (HMENU)(INT_PTR)id, g_hInst, nullptr);
    SendMessageW(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    return h;
}

static void AppendOut(const wchar_t* line) {
    int len = GetWindowTextLengthW(g_hOut);
    SendMessageW(g_hOut, EM_SETSEL, len, len);
    std::wstring s(line);
    s += L"\r\n";
    SendMessageW(g_hOut, EM_REPLACESEL, FALSE, (LPARAM)s.c_str());
}

// Кнопка «Копировать»: в буфер идёт TSV-версия данных (для Google Sheets)
static void CopyOutToClipboard() {
    if (g_tsv.empty()) {
        SetWindowTextW(g_hStatus, L"Нет данных для копирования.");
        return;
    }
    if (!OpenClipboard(g_hMain)) return;
    EmptyClipboard();
    HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, (g_tsv.size() + 1) * sizeof(wchar_t));
    if (hMem) {
        memcpy(GlobalLock(hMem), g_tsv.c_str(), (g_tsv.size() + 1) * sizeof(wchar_t));
        GlobalUnlock(hMem);
        SetClipboardData(CF_UNICODETEXT, hMem);
    }
    CloseClipboard();
    SetWindowTextW(g_hStatus, L"Данные (TSV) скопированы — вставьте в Google Sheets (Ctrl+V).");
}

static void StartMeasurement() {
    if (g_running) return;
    int sel = (int)SendMessageW(g_hCombo, CB_GETCURSEL, 0, 0);
    if (sel < 0 || sel >= (int)g_devices.size()) {
        MessageBoxW(g_hMain, L"Выберите FFB-устройство.", L"FFB Meter", MB_ICONWARNING);
        return;
    }
    auto* P = new MeasureParams();
    P->devGuid     = g_devices[sel].guid;
    P->pulseMs     = max(1,  GetEditInt(g_hMain, IDC_EDIT_PULSE, 20));
    P->forcePct    = max(1,  min(100, GetEditInt(g_hMain, IDC_EDIT_FORCE, 30)));
    P->repeats     = max(1,  GetEditInt(g_hMain, IDC_EDIT_REPEATS, 10));
    P->pauseMs = 500;// max(100, GetEditInt(g_hMain, IDC_EDIT_PAUSE, 1500));
    P->wheelDeg    = max(90, GetEditInt(g_hMain, IDC_EDIT_WHEELDEG, 900));
    P->threshDeg = max(0.01,  GetEditFloat(g_hMain, IDC_EDIT_THRESH));
    P->altDir      = SendMessageW(GetDlgItem(g_hMain, IDC_CHK_ALTDIR),BM_GETCHECK, 0, 0) == BST_CHECKED;
    P->hNotify = g_hMain;

    wchar_t hdr[256];
    swprintf(hdr, 256, L"Устройство: %s\r\nИмпульс: %d мс | Сила: %d%% | Порог: %.2f\u00B0",
             g_devices[sel].name.c_str(), P->pulseMs, P->forcePct, P->threshDeg);
    AppendOut(hdr);
    g_tsv += hdr; g_tsv += L"\r\n";

    g_stopRequest = false;
    g_running = true;
    EnableWindow(GetDlgItem(g_hMain, IDC_BTN_START), FALSE);
    EnableWindow(GetDlgItem(g_hMain, IDC_BTN_STOP), TRUE);
    g_hThread = CreateThread(nullptr, 0, MeasureThread, P, 0, nullptr);
    if (!g_hThread) {
        delete P;
        g_running = false;
        EnableWindow(GetDlgItem(g_hMain, IDC_BTN_START), TRUE);
        EnableWindow(GetDlgItem(g_hMain, IDC_BTN_STOP), FALSE);
    }
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE: {
        g_hMain = hWnd;
        g_hFontUI = CreateFontW(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                                0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
        g_hFontMono = CreateFontW(-16, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                                  0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");

        int y = 10;
        MkLabel(hWnd, L"Устройство (FFB):", 10, y, 110);
        g_hCombo = CreateWindowW(L"COMBOBOX", nullptr,
            WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
            125, y, 340, 200, hWnd, (HMENU)IDC_COMBO_DEV, g_hInst, nullptr);
        SendMessageW(g_hCombo, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        MkBtn(hWnd, IDC_BTN_REFRESH, L"Обновить", 475, y - 1, 90, 24);

        y += 36;
        int x = 10;
        MkLabel(hWnd, L"Импульс, мс:", x, y, 80); x += 85; MkEdit(hWnd, IDC_EDIT_PULSE, L"20", x, y, 50); x += 65;
        MkLabel(hWnd, L"Сила, %:",     x, y, 50);  x += 55; MkEdit(hWnd, IDC_EDIT_FORCE,   L"100", x, y, 50); x += 65;
        MkLabel(hWnd, L"Повторы:",     x, y, 60); x += 65; MkEdit(hWnd, IDC_EDIT_REPEATS, L"6", x, y, 50); x += 65;
        //MkLabel(hWnd, L"Пауза, мс:",     x, y, 70);  x += 70; MkEdit(hWnd, IDC_EDIT_PAUSE,   L"500", x, y, 50); x += 65;
        MkLabel(hWnd, L"Руль, град:",    x,  y, 70); x += 75; MkEdit(hWnd, IDC_EDIT_WHEELDEG, L"900", x, y, 50); x += 65;
        MkLabel(hWnd, L"Порог, град:",   x, y, 80);  x += 85; MkEdit(hWnd, IDC_EDIT_THRESH,   L"0.01", x, y, 50); x += 65;

        {
            HWND chk = CreateWindowW(L"BUTTON", L"Чередовать направление (±)",
                WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                x, y, 200, 22, hWnd, (HMENU)IDC_CHK_ALTDIR, g_hInst, nullptr);
            SendMessageW(chk, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
            SendMessageW(chk, BM_SETCHECK, BST_CHECKED, 0);

            x += 220;
        }

        y += 34;
        MkBtn(hWnd, IDC_BTN_START, L"▶ Старт",      10,  y, 110, 30);
        MkBtn(hWnd, IDC_BTN_STOP,  L"■ Стоп",       130, y, 110, 30);
        MkBtn(hWnd, IDC_BTN_COPY,  L"Копировать",   250, y, 110, 30);
        MkBtn(hWnd, IDC_BTN_CLEAR, L"Очистить",     370, y, 110, 30);
        EnableWindow(GetDlgItem(hWnd, IDC_BTN_STOP), FALSE);

        g_hOut = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
            WS_CHILD | WS_VISIBLE | WS_VSCROLL | //WS_HSCROLL | 
            ES_MULTILINE | ES_AUTOVSCROLL | ES_AUTOHSCROLL | ES_READONLY,
            0, 0, 10, 10, hWnd, (HMENU)IDC_EDIT_OUT, g_hInst, nullptr);
        SendMessageW(g_hOut, WM_SETFONT, (WPARAM)g_hFontMono, TRUE);

        //------------------------------
        RegisterGraphClass(g_hInst);

        g_hGraphLbl = CreateWindowW(L"STATIC", L"Нет графиков",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            0, 0, 10, 10, hWnd, nullptr, g_hInst, nullptr);
        SendMessageW(g_hGraphLbl, WM_SETFONT, (WPARAM)g_hFontUI, TRUE); // тот же шрифт, что у остального UI

        g_hCopyright = CreateWindowW(L"STATIC", L"©2026, @morskoyzmey",
            WS_CHILD | WS_VISIBLE | SS_RIGHT,
            0, 0, 10, 10, hWnd, nullptr, g_hInst, nullptr);
        SendMessageW(g_hCopyright, WM_SETFONT, (WPARAM)g_hFontUI, TRUE); // тот же шрифт, что у остального UI


        g_hGraph = CreateWindowW(L"FfbLatGraphWnd", nullptr,
            WS_CHILD | WS_VISIBLE | WS_BORDER,
            0, 0, 10, 10, hWnd, nullptr, g_hInst, nullptr);

        //------------------------------

        g_hStatus = CreateWindowW(L"STATIC", L"Готов.",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            0, 0, 800, 20, hWnd, (HMENU)IDC_STATIC_STATUS, g_hInst, nullptr);
        SendMessageW(g_hStatus, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);

        HRESULT hr = DirectInput8Create(g_hInst, DIRECTINPUT_VERSION,
                                        IID_IDirectInput8W, (void**)&g_pDI, nullptr);
        if (FAILED(hr)) {
            MessageBoxW(hWnd, L"Не удалось инициализировать DirectInput8.",
                        L"FFB Meter", MB_ICONERROR);
        }
        RefreshDevices();
        return 0;
    }

    case WM_SIZE: {
        RECT rc; GetClientRect(hWnd, &rc);
        int w = rc.right, h = rc.bottom;

        const int PAD = 10;
        int baseY = 130;
        float g_part = 0.55;

        MoveWindow(g_hGraphLbl, PAD, baseY, 260, 20, TRUE);
        MoveWindow(g_hGraph, PAD, baseY + 25, w* g_part - PAD, h - 25 - 30 - baseY, TRUE);

        MoveWindow(g_hOut, w * g_part + PAD, baseY + 25, w * (1.0 - g_part) - PAD * 2, h - 25 - 30 - baseY, TRUE);

        RECT orc;
        GetClientRect(g_hOut, &orc);
        orc.left += 8;
        orc.top += 8;
        orc.right -= 8;
        orc.bottom -= 8;
        SendMessage(g_hOut, EM_SETRECT, 0, (LPARAM)&orc);

        MoveWindow(g_hStatus, 10, rc.bottom - 25, rc.right - 250, 20, TRUE);
        MoveWindow(g_hCopyright, rc.right - 200 - PAD, rc.bottom - 25, 200, 20, TRUE);
        return 0;
    }

    case WM_APP_CLEARGRAPH: {
        ClearGraphsList();
        return 0;
    }

    case WM_APP_ADDGRAPH: {
        GraphData* gd = (GraphData*)lp;
        gd->ltDeg = (float)ComputeLinThreshold(gd);
        g_graphs.push_back(gd);
        // автопереход на только что добавленный график
        SetCurGraph((int)g_graphs.size() - 1);
        return 0;
    }

    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_BTN_PREV: SetCurGraph(g_curGraph - 1); break;
        case IDC_BTN_NEXT: SetCurGraph(g_curGraph + 1); break;
        case IDC_BTN_REFRESH: RefreshDevices(); break;
        case IDC_BTN_START:   StartMeasurement(); break;
        case IDC_BTN_STOP:    g_stopRequest = true; break;
        case IDC_BTN_COPY:    CopyOutToClipboard(); break;
        case IDC_BTN_CLEAR:
            SetWindowTextW(g_hOut, L"");
            g_tsv.clear();

            ClearGraphsList();

            break;
        }
        return 0;

    case WM_APP_ADDROW: {
        wchar_t* s = (wchar_t*)lp;
        if (s) { AppendOut(s); free(s); }
        return 0;
    }
    case WM_APP_ADDTSV: {
        wchar_t* s = (wchar_t*)lp;
        if (s) { g_tsv += s; g_tsv += L"\r\n"; free(s); }
        return 0;
    }
    case WM_APP_STATUS: {
        wchar_t* s = (wchar_t*)lp;
        if (s) { SetWindowTextW(g_hStatus, s); free(s); }
        return 0;
    }
    case WM_APP_DONE:
        g_running = false;
        if (g_hThread) { CloseHandle(g_hThread); g_hThread = nullptr; }
        EnableWindow(GetDlgItem(hWnd, IDC_BTN_START), TRUE);
        EnableWindow(GetDlgItem(hWnd, IDC_BTN_STOP), FALSE);
        AppendOut(L"");
        g_tsv += L"\r\n";
        return 0;

    case WM_CLOSE:
        g_stopRequest = true;
        if (g_hThread) {
            WaitForSingleObject(g_hThread, 5000);
            CloseHandle(g_hThread);
            g_hThread = nullptr;
        }
        DestroyWindow(hWnd);
        return 0;

    case WM_DESTROY:
        for (auto* g : g_graphs) delete g;
        g_graphs.clear();

        if (g_pDI) { g_pDI->Release(); g_pDI = nullptr; }
        if (g_hFontUI)  DeleteObject(g_hFontUI);
        if (g_hFontMono) DeleteObject(g_hFontMono);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wp, lp);
}

static void CenterWindowOnScreen(HWND hWnd)
{
    RECT rcWnd;
    GetWindowRect(hWnd, &rcWnd);
    int w = rcWnd.right - rcWnd.left;
    int h = rcWnd.bottom - rcWnd.top;

    // Монитор, ближайший к окну (на старте это обычно primary,
    // но если окно создано с явными координатами - учтёт нужный)
    HMONITOR hMon = MonitorFromWindow(hWnd, MONITOR_DEFAULTTONEAREST);

    MONITORINFO mi = { sizeof(mi) };
    GetMonitorInfo(hMon, &mi);

    // rcWork - рабочая область монитора, БЕЗ панели задач
    RECT rcWork = mi.rcWork;
    int screenW = rcWork.right - rcWork.left;
    int screenH = rcWork.bottom - rcWork.top;

    int x = rcWork.left + (screenW - w) / 2;
    int y = rcWork.top + (screenH - h) / 2;

    SetWindowPos(hWnd, nullptr, x, y, 0, 0,
        SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
}
// ------------------------------------------------------------- WinMain ------
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int nShow) {
    g_hInst = hInst;
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

    WNDCLASSW wc = {};
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInst;
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wc.lpszClassName = L"FFBMeterWnd";
    wc.hIcon         = LoadIcon(hInst, MAKEINTRESOURCE(IDI_APP_ICON));
    RegisterClassW(&wc);
    
    HWND hWnd = CreateWindowW(L"FFBMeterWnd",
        L"FFB Meter - 0.1.5",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1500, 900,
        nullptr, nullptr, hInst, nullptr);

    CenterWindowOnScreen(hWnd);

    ShowWindow(hWnd, nShow);
    UpdateWindow(hWnd);

    MSG m;
    while (GetMessageW(&m, nullptr, 0, 0) > 0) {
        TranslateMessage(&m);
        DispatchMessageW(&m);
    }
    CoUninitialize();
    return 0;
}

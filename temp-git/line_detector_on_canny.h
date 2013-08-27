#ifndef LINE_DETECTOR_ON_CANNY_H
#define LINE_DETECTOR_ON_CANNY_H


#include "Additional_Functions.h"
#include "Line_Detector.h"
#include "ConfigurationManager.h"
#include "debuglog.h"
#include "Color_Analyzer.h"
#include "Corner_Detection.h"
//#include "Wall_Detector_New.h"
using namespace std;
enum Border{
NotOnBorder,
Left,
Top,
Right,
Bottom
};
struct Line{
cv::Point2i p2iEnd1;
cv::Point2i p2iEnd2;
};
typedef vector<cv::Point2i> RefDigitalLine;
typedef vector<RefDigitalLine> Window;
class Line_Detector_On_Canny
{
private:
    cv::Mat matCannyImage;
    map<pair<int,int>, bool> mapCannyPixelPartOfLine;
    int iWindowSize, iCorrelationPercentage;
    int iImageWidth, iImageHeight;
    Window refWindow;
    RefDigitalLine windowBottomLine, windowTopLine, windowLeftLine, windowRightLine;
    RefDigitalLine refWindowBottomLine, refWindowTopLine,refWindowLeftLine, refWindowRightLine;
    string strResultpath;
    int CreateWindow();
    void ReadEdgePixels();
    int FindWindowEdgeOfThePixel(cv::Point2i p2iInputPixel, Border& pixelEdge);
    int GetOppositEdgeOfPixelEdge(cv::Point2i point, Border& border);
    int GetPixelsOnLine(cv::Point2i p2iEnd1, cv::Point2i p2iEnd2, RefDigitalLine& RefDigLine);
    int FindDigitalLinesInAWindow(Window windowAtPixel, vector<RefDigitalLine>& vlLines,vector<int>& viRefLineIndices);
    int MoveWindowAtAPixel(cv::Point2i p2iPixel,Border aligmentBorder,Window& windowAtPixel);
    int MoveWindowAtAPixelAndFindLines(cv::Point2i p2iPixel,Border aligmentBorder,vector<Line>& vDigitalLines );
public:

    int GetTwoDWindow(cv::Point2i p2iPoint, int iWindowLength, cv::Vector<cv::Point2i> &vp2iTwoDWindow);
    cv::Vec2i GetOneDWindow(int iRefPoint, int iWindowLength, bool bFlgIsXAxis);
    int CheckWindowAroundAPixel(cv::Point2i p2iPixel,Border lineBorder,vector<Line>& vv4iLines);
    Line_Detector_On_Canny(cv::Mat matCannyImage,string strResult);
    int Initialise(int iWindowSizeInput, int iCorrelationPercentInput);
    int DetectLines(vector<Line>& vlinesOnCanny );
    int DetectLinesAlongBorders(vector<Line>& vlinesOnCanny);
    int DetectLinesAlongBorder(Border,vector<Line>& v4iLinesDetected);
    int DetectLinesAroundCannyPixels(vector<Line>& vlinesOnCanny);
    bool DetectPixelsAlongALine(Line v4iBorderLine, bool isLineVertical,vector<cv::Point2i>& vp2iPixels);
    int FindCorrelatedLineAlongPixel(cv::Point2i p2iPoint, cv::Vec4i v4iLine);
};
#endif

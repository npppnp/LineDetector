#include "line_detector_on_canny.h"

Line_Detector_On_Canny::Line_Detector_On_Canny(cv::Mat matCannyImageInput, string strResult)
{
    matCannyImage = matCannyImageInput;
    strResultpath = strResult;

}
void Line_Detector_On_Canny::ReadEdgePixels()
{
    int iNoOfRows = matCannyImage.size().height;
    int iNoOfColumns = matCannyImage.size().width;
    uchar ucColor;
    cout<<" iImageWidth "<<iImageWidth<<" iImageHeight "<<iImageHeight;
    for(int i = 0; i< iNoOfRows; i++)
    {

        for(int j = 0; j< iNoOfColumns; j++)
        {
            ucColor = matCannyImage.at<uchar>(i,j);

            if(ucColor== 255)
            {
                pair<int,int> mapPoint(j,i);
               mapCannyPixelPartOfLine.insert(std::pair<std::pair<int,int>,bool>(mapPoint,false));
               if((j > (iImageWidth-1))||(i > (iImageHeight-1)))
               cout<<"\n in ReadEdgePixels at "<<j<<""<<i;
            }
        }
    }

}
cv::Vec2i Line_Detector_On_Canny::GetOneDWindow(int iRefPoint, int iWindowLength, bool bFlgIsXAxis)
{

    cv::Vec2i v2iWindow;
    int iHalfOfWindow = iWindowLength/2;

    if(iHalfOfWindow < iRefPoint)
        v2iWindow[0] = (iRefPoint - iHalfOfWindow);
    else
        v2iWindow[0] = 0;

    if(bFlgIsXAxis)
    {
        if((iHalfOfWindow+iRefPoint)> iImageWidth)
            v2iWindow[1] = (iImageWidth - 1);
        else
            v2iWindow[1] = iHalfOfWindow+iRefPoint;
    }
    else
    {
        if((iHalfOfWindow+iRefPoint)> iImageHeight)
            v2iWindow[1] = (iImageHeight - 1);
        else
            v2iWindow[1] = iHalfOfWindow+iRefPoint;
    }
    return v2iWindow;
}
int Line_Detector_On_Canny::GetTwoDWindow(cv::Point2i p2iPoint, int iWindowLength, cv::Vector<cv::Point2i> &vp2iTwoDWindow)
{

   cv::Vec2i v2iTempOneDWindowX, v2iTempOneDWindowY;
//   vp2iTwoDWindow.reserve(2);
   cv::Point2i p2iTempPoint;

   v2iTempOneDWindowX = GetOneDWindow(p2iPoint.x, iWindowLength, true);
   v2iTempOneDWindowY = GetOneDWindow(p2iPoint.y, iWindowLength, false);
   p2iTempPoint.x = v2iTempOneDWindowX[0];
   p2iTempPoint.y = v2iTempOneDWindowY[0];
   vp2iTwoDWindow.push_back(p2iTempPoint);

   p2iTempPoint.x = v2iTempOneDWindowX[1];
   p2iTempPoint.y = v2iTempOneDWindowY[0];
   vp2iTwoDWindow.push_back(p2iTempPoint);

   p2iTempPoint.x = v2iTempOneDWindowX[1];
   p2iTempPoint.y = v2iTempOneDWindowY[1];
   vp2iTwoDWindow.push_back(p2iTempPoint);

   p2iTempPoint.x = v2iTempOneDWindowX[0];
   p2iTempPoint.y = v2iTempOneDWindowY[1];
   vp2iTwoDWindow.push_back(p2iTempPoint);

}
int Line_Detector_On_Canny::FindDigitalLinesInAWindow(Window windowAtPixel, vector<RefDigitalLine>& vlLines, vector<int>& viRefLineIndices)
{
    cv::Point2i p2iRefPixel, p2iWindowPixel;
    uchar ucColor;
    for(int i =0; i < windowAtPixel.size(); i++)
    {
        Line lineFound;
        RefDigitalLine digLine = windowAtPixel[i];
        RefDigitalLine digLineAtPixel;
        for(int j=0; j<digLine.size();j++)
        {
//            cout<<"\n the ref line found "<<digLine;
            p2iRefPixel = digLine[j];
            ucColor = matCannyImage.at<uchar>(p2iRefPixel.y,p2iRefPixel.x);
            pair<int,int> mapPoint(p2iRefPixel.x,p2iRefPixel.y);
            try
            {
            //check if the pixel is not a part of a line before
                if((ucColor == 255)&&(mapCannyPixelPartOfLine[mapPoint] == false))
                {
                    digLineAtPixel.push_back(p2iRefPixel);
//                    cout<<"\n****************** pixel on line "<<p2iRefPixel;
                }
            }
            catch(exception)
            {

            }
        }

        if(digLineAtPixel.size()> iWindowSize/4)
        {
            //set flag to identify pixel being a part of a line
            for(int j=0; j<(digLine.size()-2);j++)
            {
                p2iRefPixel = digLine[j];
                pair<int,int> mapPoint(p2iRefPixel.x,p2iRefPixel.y);
                mapCannyPixelPartOfLine[mapPoint] = true;
            }
            lineFound.p2iEnd1 = digLineAtPixel[0];
            lineFound.p2iEnd2 = digLineAtPixel[digLineAtPixel.size()-1];
            vlLines.push_back(digLineAtPixel);
            viRefLineIndices.push_back(i);
//            cout<<"\n found digital line at  : "<<lineFound.p2iEnd1<<lineFound.p2iEnd2;
//            cout<<" size of line "<<digLineAtPixel.size();
//            cout<<" the line found "<<digLineAtPixel;
//            cout<<"\n the ref line found "<<digLine;
        }
    }

}
int Line_Detector_On_Canny::MoveWindowAtAPixelAndFindLines(cv::Point2i p2iPixel,Border aligmentBorder,vector<Line>& vDigitalLines )
{
    cv::Point2i p2iRefPixel, p2iWindowPixel;
    int iWindowAlignmentMargin;
    bool flgAlingWindowVertically = false;
    int iXOffset, iYOffset;
    bool flgMoveWindowLineAtPixel;
//    cout<<"\n moving window around pixel "<<p2iPixel;
    iXOffset = p2iPixel.x;
    iYOffset = p2iPixel.y;
    if(aligmentBorder == NotOnBorder)
    {
        iXOffset -= iWindowSize/2;
        iYOffset -= iWindowSize/2;
    }
    else if((aligmentBorder == Left)||(aligmentBorder == Right))
    {
        flgAlingWindowVertically = true;
        if((p2iPixel.y > (iWindowSize/2 -1))||(p2iPixel.y == (iWindowSize/2-1)))
            iYOffset -= iWindowSize/2;
        else
            iYOffset -= p2iPixel.y;
        if(aligmentBorder == Right)
            iXOffset = p2iPixel.x - (iWindowSize-1);//(iImageWidth-1) - (iWindowSize-1);

    }
    else if((aligmentBorder == Top)||(aligmentBorder == Bottom))
    {
        if((p2iPixel.x > (iWindowSize/2 -1))||(p2iPixel.x == (iWindowSize/2-1)))
            iXOffset -= iWindowSize/2;
        else
            iXOffset -= p2iPixel.x;
        if(aligmentBorder == Bottom)
            iYOffset = p2iPixel.y - (iWindowSize-1);
    }
    if(iXOffset < 0)
      iXOffset = 0;
    if(iYOffset < 0)
      iYOffset = 0;
//    cout<<"\n Alignment margin "<<iWindowAlignmentMargin;

    int iRefWindowSize = refWindow.size();
    int iNoOfPixelsInLine, iNoOfWHitePixelsFoundOnWindowLine ;
    for(int i =0; i < iRefWindowSize; i++)
    {

        RefDigitalLine digLine = refWindow[i];
        RefDigitalLine digLineAtPixel;
        flgMoveWindowLineAtPixel = false;
        iNoOfPixelsInLine = digLine.size();
        int iNoOfMinWhitePixels = iNoOfPixelsInLine/4;
        iNoOfWHitePixelsFoundOnWindowLine = 0;
        for(int j=0; j<iNoOfPixelsInLine;j++)
        {
            p2iRefPixel = digLine[j];
            p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
            p2iWindowPixel.y = iYOffset + p2iRefPixel.y;


            //refine this boundary check logic
            if((aligmentBorder == Left)||(aligmentBorder == Top))
            {
                if((p2iWindowPixel.x == (iImageWidth-1))||(p2iWindowPixel.y == (iImageHeight-1)))
                    break;
            }
            if((aligmentBorder == Right)||(aligmentBorder == Bottom))
            {
                if((p2iWindowPixel.x == 0)||(p2iWindowPixel.y == 0))
                    break;
            }
            //check if there is a white pixel at the WindowLine
//            if(!flgMoveWindowLineAtPixel)
            {
                if((matCannyImage.at<uchar>(p2iWindowPixel.y,p2iWindowPixel.x)==255)
                        &&(mapCannyPixelPartOfLine[std::pair<int,int>(p2iWindowPixel.x,p2iWindowPixel.y)] == false))
                {
                    iNoOfWHitePixelsFoundOnWindowLine++;

                    digLineAtPixel.push_back(p2iWindowPixel);
//                  flgMoveWindowLineAtPixel = true;
                }
            }
        }

        if((iNoOfWHitePixelsFoundOnWindowLine>iNoOfMinWhitePixels)&&
                (iNoOfWHitePixelsFoundOnWindowLine>2))
        {
            Line tempLineInWindow;
            int iSizeOfDigLine = digLineAtPixel.size();
            iSizeOfDigLine--;
            tempLineInWindow.p2iEnd1.x = digLineAtPixel[0].x;
            tempLineInWindow.p2iEnd1.y = digLineAtPixel[0].y;
            tempLineInWindow.p2iEnd2.x = digLineAtPixel[iSizeOfDigLine].x;
            tempLineInWindow.p2iEnd2.y = digLineAtPixel[iSizeOfDigLine].y;
            vDigitalLines.push_back(tempLineInWindow);
            cv::Point pointOnLine;
            RefDigitalLine::iterator itDigLine, itDigLineEnd;
            itDigLine = digLineAtPixel.begin();
            itDigLineEnd = digLineAtPixel.end();
            for (; itDigLine != itDigLineEnd;itDigLine++) {
                mapCannyPixelPartOfLine[std::pair<int,int>((*itDigLine).x,(*itDigLine).y)] = true;
            }
        }
    }

//    RefDigitalLine digLine = refWindowLeftLine;
////    cout<<"\n iXOffset "<<iXOffset<<" iYOffset "<<iYOffset;
////    cout<<"\n old windowLeftLine "<<digLine;
//    windowLeftLine.erase(windowLeftLine.begin(),windowLeftLine.end());
//    for(int j=0; j<digLine.size();j++)
//    {
//        p2iRefPixel = digLine[j];
//        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
//        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
//        windowLeftLine.push_back(p2iWindowPixel);
//    }
////    cout<<"\n new windowLeftLine "<<windowLeftLine;
//    digLine = refWindowRightLine;
//    windowRightLine.erase(windowRightLine.begin(),windowRightLine.end());
//    for(int j=0; j<digLine.size();j++)
//    {
//        p2iRefPixel = digLine[j];
//        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
//        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
//        windowRightLine.push_back(p2iWindowPixel);

//    }
////        cout<<"\n new windowRightLine "<<windowRightLine;
//    digLine = refWindowTopLine;
//    windowTopLine.erase(windowTopLine.begin(),windowTopLine.end());
//    for(int j=0; j<digLine.size();j++)
//    {
//        p2iRefPixel = digLine[j];
//        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
//        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
//        windowTopLine.push_back(p2iWindowPixel);

//    }
////    cout<<"\n new windowTopLine "<<windowTopLine;
//    digLine = refWindowBottomLine;
//    windowBottomLine.erase(windowBottomLine.begin(),windowBottomLine.end());
//    for(int j=0; j<digLine.size();j++)
//    {
//        p2iRefPixel = digLine[j];
//        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
//        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
//        windowBottomLine.push_back(p2iWindowPixel);

//    }
//    cout<<"\n new windowBottomLine "<<windowBottomLine;
    //    cout<<"\n Window first and last lines "<<windowAtPixel[0]<<windowAtPixel[windowAtPixel.size()-1];
    if(vDigitalLines.size()==0)
        return -1;
    else
        return 1;
}
int Line_Detector_On_Canny::MoveWindowAtAPixel(cv::Point2i p2iPixel,Border aligmentBorder,Window& windowAtPixel )
{
    cv::Point2i p2iRefPixel, p2iWindowPixel;
    int iWindowAlignmentMargin;
    bool flgAlingWindowVertically = false;
    int iXOffset, iYOffset;
    bool flgMoveWindowLineAtPixel;
//    cout<<"\n moving window around pixel "<<p2iPixel;
    iXOffset = p2iPixel.x;
    iYOffset = p2iPixel.y;
    if(aligmentBorder == NotOnBorder)
    {
        iXOffset -= iWindowSize/2;
        iYOffset -= iWindowSize/2;
    }
    else if((aligmentBorder == Left)||(aligmentBorder == Right))
    {
        flgAlingWindowVertically = true;
        if((p2iPixel.y > (iWindowSize/2 -1))||(p2iPixel.y == (iWindowSize/2-1)))
            iYOffset -= iWindowSize/2;
        else
            iYOffset -= p2iPixel.y;
        if(aligmentBorder == Right)
            iXOffset = p2iPixel.x - (iWindowSize-1);//(iImageWidth-1) - (iWindowSize-1);

    }
    else if((aligmentBorder == Top)||(aligmentBorder == Bottom))
    {
        if((p2iPixel.x > (iWindowSize/2 -1))||(p2iPixel.x == (iWindowSize/2-1)))
            iXOffset -= iWindowSize/2;
        else
            iXOffset -= p2iPixel.x;
        if(aligmentBorder == Bottom)
            iYOffset = p2iPixel.y - (iWindowSize-1);
    }
    if(iXOffset < 0)
      iXOffset = 0;
    if(iYOffset < 0)
      iYOffset = 0;
//    cout<<"\n Alignment margin "<<iWindowAlignmentMargin;

    int iRefWindowSize = refWindow.size();
    int iNoOfPixelsInLine ;
    for(int i =0; i < iRefWindowSize; i++)
    {
        RefDigitalLine digLine = refWindow[i];
        RefDigitalLine digLineAtPixel;
        flgMoveWindowLineAtPixel = false;
        iNoOfPixelsInLine = digLine.size();
        for(int j=0; j<iNoOfPixelsInLine;j++)
        {
            p2iRefPixel = digLine[j];
            p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
            p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
            digLineAtPixel.push_back(p2iWindowPixel);

            //refine this boundary check logic
            if((aligmentBorder == Left)||(aligmentBorder == Top))
            {
                if((p2iWindowPixel.x == (iImageWidth-1))||(p2iWindowPixel.y == (iImageHeight-1)))
                    break;
            }
            if((aligmentBorder == Right)||(aligmentBorder == Bottom))
            {
                if((p2iWindowPixel.x == 0)||(p2iWindowPixel.y == 0))
                    break;
            }
            //check if there is a white pixel at the WindowLine
            if(!flgMoveWindowLineAtPixel)
            {
                if(matCannyImage.at<uchar>(p2iWindowPixel.y,p2iWindowPixel.x)==255)
                    flgMoveWindowLineAtPixel = true;
            }
        }

        if((digLineAtPixel.size()>1)&&(flgMoveWindowLineAtPixel == true))
        {
            windowAtPixel.push_back(digLineAtPixel);
        }
    }

    RefDigitalLine digLine = refWindowLeftLine;
//    cout<<"\n iXOffset "<<iXOffset<<" iYOffset "<<iYOffset;
//    cout<<"\n old windowLeftLine "<<digLine;
    windowLeftLine.erase(windowLeftLine.begin(),windowLeftLine.end());
    for(int j=0; j<digLine.size();j++)
    {
        p2iRefPixel = digLine[j];
        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
        windowLeftLine.push_back(p2iWindowPixel);
    }
//    cout<<"\n new windowLeftLine "<<windowLeftLine;
    digLine = refWindowRightLine;
    windowRightLine.erase(windowRightLine.begin(),windowRightLine.end());
    for(int j=0; j<digLine.size();j++)
    {
        p2iRefPixel = digLine[j];
        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
        windowRightLine.push_back(p2iWindowPixel);

    }
//        cout<<"\n new windowRightLine "<<windowRightLine;
    digLine = refWindowTopLine;
    windowTopLine.erase(windowTopLine.begin(),windowTopLine.end());
    for(int j=0; j<digLine.size();j++)
    {
        p2iRefPixel = digLine[j];
        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
        windowTopLine.push_back(p2iWindowPixel);

    }
//    cout<<"\n new windowTopLine "<<windowTopLine;
    digLine = refWindowBottomLine;
    windowBottomLine.erase(windowBottomLine.begin(),windowBottomLine.end());
    for(int j=0; j<digLine.size();j++)
    {
        p2iRefPixel = digLine[j];
        p2iWindowPixel.x = iXOffset + p2iRefPixel.x;
        p2iWindowPixel.y = iYOffset + p2iRefPixel.y;
        windowBottomLine.push_back(p2iWindowPixel);

    }
//    cout<<"\n new windowBottomLine "<<windowBottomLine;
    //    cout<<"\n Window first and last lines "<<windowAtPixel[0]<<windowAtPixel[windowAtPixel.size()-1];
    if(windowAtPixel.size()==0)
        return -1;
    else
        return 1;
}
//return edge opposite to the edge on which pixel lies
int Line_Detector_On_Canny::GetOppositEdgeOfPixelEdge(cv::Point2i p2iInputPixel, Border& OppositEdgeToPixelEdge)
{
    Border pixelBorder;
    int iReturn = 1;
    if(FindWindowEdgeOfThePixel(p2iInputPixel, pixelBorder) != -1)
    {
        switch(pixelBorder)
        {
            case Left:
                OppositEdgeToPixelEdge = Right;
                break;
            case Right:
                OppositEdgeToPixelEdge = Left;
                break;
            case Top:
                OppositEdgeToPixelEdge = Bottom;
                break;
            case Bottom:
                OppositEdgeToPixelEdge = Top;
                break;
        }
    }
    else
    {
        OppositEdgeToPixelEdge = NotOnBorder;
//        iReturn = -1;
    }
//cout<<"\n opposit edge of the pixel"<<OppositEdgeToPixelEdge;
    return iReturn;
}
//finds if the pixel lies on the edge of the window used for line detectn
int Line_Detector_On_Canny::FindWindowEdgeOfThePixel(cv::Point2i p2iInputPixel, Border& pixelEdge)
{

    cv::Point2i p2iRefPixel;
    RefDigitalLine digLine ;
    Border tempBorder = NotOnBorder;
        int iSize;
    if(tempBorder == NotOnBorder)
    {
        digLine = windowLeftLine;
        iSize = windowLeftLine.size();
        for(int i = 0; i < iSize;i++)
        {
            p2iRefPixel = digLine[i];
            if(p2iRefPixel == p2iInputPixel)
            {
                tempBorder = Left;
                break;
            }

        }
    }
    if(tempBorder == NotOnBorder)
    {
        digLine = windowRightLine;
        iSize = windowRightLine.size();
        for(int i = 0; i < iSize;i++)
        {
            p2iRefPixel = digLine[i];
            if(p2iRefPixel == p2iInputPixel)
            {
                tempBorder = Right;
            }

        }
    }
    if(tempBorder == NotOnBorder)
    {
        digLine = windowTopLine;
        iSize = windowTopLine.size();
        for(int i = 0; i < iSize;i++)
        {
            p2iRefPixel = digLine[i];
            if(p2iRefPixel == p2iInputPixel)
            {
                tempBorder = Top;
            }

        }
    }
    if(tempBorder == NotOnBorder)
    {
        digLine = windowBottomLine;
        iSize = windowBottomLine.size();
        for(int i = 0; i < iSize;i++)
        {
            p2iRefPixel = digLine[i];
            if(p2iRefPixel == p2iInputPixel)
            {
                tempBorder = Bottom;
            }

        }
    }
    pixelEdge = tempBorder;
//    cout<<"\n edge of the pixel"<<pixelEdge;
    if(tempBorder != NotOnBorder)
        return 1;
    else
        return -1;

}
int Line_Detector_On_Canny::CheckWindowAroundAPixel(cv::Point2i p2iPixel,Border lineBorder,vector<Line>& digLinesFound)
{
    cv::Vector<cv::Point2i> vp2iTwoDWindow;
    vector<RefDigitalLine> digLinesFoundTemp;
    //GetTwoDWindow(p2iPixel,iWindowSize, vp2iTwoDWindow);
    cv::Mat matTempImage;
    Window windowAtPixel;
    vector<int> viRefLineIndexes;
    cv::cvtColor(matCannyImage,matTempImage,CV_GRAY2BGR);
    circle(matTempImage,p2iPixel,5,cv::Scalar(255,0,0),2,8,0);
    //cv::polylines(matTempImage, vp2iTwoDWindow, true, cv::Scalar(255,0,0), 1, 8, 0);

    int iLinesFoundCount = digLinesFound.size();
//    if(MoveWindowAtAPixel(p2iPixel,lineBorder, windowAtPixel) != -1)
//    {
//        FindDigitalLinesInAWindow(windowAtPixel,digLinesFoundTemp,viRefLineIndexes);
//    }
    MoveWindowAtAPixelAndFindLines(p2iPixel,lineBorder,digLinesFound );
    //remove duplicate lines
    RefDigitalLine tempLine1, tempLine2, LineToBeConsidered ;
    Line lineFound, lineTemp;
    vector<int> viRefLineIndices;
    int iNoOfLinesFoundAroundPixel = digLinesFoundTemp.size();
    int iSizeOfRefLine;
    int iNoOfLinesDetected = digLinesFound.size();
    for(int i =0; i< iNoOfLinesFoundAroundPixel;i++)
    {
        bool flgLineCopied;
        flgLineCopied = false;
        tempLine1 = digLinesFoundTemp[i];
        LineToBeConsidered = tempLine1;
        iSizeOfRefLine = tempLine1.size();
        //check if another line with same ends exists
        for(int j =i; j< iNoOfLinesFoundAroundPixel;j++)
        {
            tempLine2 = digLinesFoundTemp[j];
            if(((tempLine1[0] == tempLine2[0])&&(tempLine1[tempLine1.size()-1] == tempLine2[tempLine2.size()-1]))||
               ((tempLine1[tempLine1.size()-1] == tempLine2[0])&&(tempLine1[0] == tempLine2[tempLine2.size()-1])))
            {
                if(iSizeOfRefLine<tempLine2.size())
                    LineToBeConsidered = tempLine2;
            }
        }
        lineFound.p2iEnd1 = LineToBeConsidered[0];
        lineFound.p2iEnd2 = LineToBeConsidered[LineToBeConsidered.size()-1];
        for(int k = 0; k < iNoOfLinesDetected; k++)
        {
            lineTemp = digLinesFound[k];
            if(((lineTemp.p2iEnd1 == lineFound.p2iEnd1)&&(lineTemp.p2iEnd2 == lineFound.p2iEnd2))||
               ((lineTemp.p2iEnd2 == lineFound.p2iEnd1)&&(lineTemp.p2iEnd1 == lineFound.p2iEnd2)))
                    flgLineCopied = true;
        }
        if(flgLineCopied == false)
        {
            digLinesFound.push_back(lineFound);
//            int i = viRefLineIndexes[i];
//            cout<<"\n window line index "<<i;
//            viRefLineIndices.push_back(i);

        }
    }

//    <<"\n size of digLinesFound "<<digLinesFound.size()<<" size of viRefLineIndices "<<viRefLineIndices.size()<<" iLinesFoundCount "<<iLinesFoundCount;
//********** code for animation *****************
//        for(int k = 0; k < digLinesFound.size(); k++)
//    {
//        lineTemp = digLinesFound[k];
////        cout<<"\n Lines found "<<digLinesFound[k].p2iEnd1<<digLinesFound[k].p2iEnd2;
//         cv::line(matTempImage,lineTemp.p2iEnd1,lineTemp.p2iEnd2,cv::Scalar(0,255,0),1,8,0);
//    }

//    cv::imshow("Windows found",matTempImage);
//    cv::waitKey();
//    cout<<"\n*********************** entring recursion "<<iLinesFoundCount;
    if(lineBorder != NotOnBorder)
    {


        for(int k = iLinesFoundCount; k < digLinesFound.size(); k++)
    {
        lineTemp = digLinesFound[k];
        pair<int,int> mapPoint1(digLinesFound[k].p2iEnd1.x,digLinesFound[k].p2iEnd1.y);
        pair<int,int> mapPoint2(digLinesFound[k].p2iEnd2.x,digLinesFound[k].p2iEnd2.y);
        Border newEdgeOfWindow;
        cv::Point2i p2iNewPointForWindow;
        cv::Point2i p2iNewPointEdgDetectn;
        pair<int,int> mapPointToRefer;

        if((lineBorder == Left)||(lineBorder == Top))
        {
            mapCannyPixelPartOfLine[mapPoint1] = true;
            p2iNewPointForWindow = digLinesFound[k].p2iEnd2;
//            cout<<"\n THE LINE 1 index is "<<k<<" siz eof viRefLineIndices "<<viRefLineIndices.size();
//            int z = viRefLineIndices[k-iLinesFoundCount];
//            cout<<"\n window line index "<<z;
//            RefDigitalLine windowLine = windowAtPixel[z];
//            int i = windowLine.size();
//            p2iNewPointEdgDetectn = windowLine[i-1];
//            cout<<"\n window line end "<<p2iNewPointEdgDetectn;
            mapPointToRefer = mapPoint2;

        }
        else
        {

            mapCannyPixelPartOfLine[mapPoint2] = true;
            p2iNewPointForWindow = digLinesFound[k].p2iEnd1;
//            cout<<"\n THE LINE 1 index is "<<k<<" siz eof viRefLineIndices "<<viRefLineIndices.size();
//            int z = viRefLineIndices[k-iLinesFoundCount];
//            cout<<"\n window line index "<<z;
//            RefDigitalLine windowLine = windowAtPixel[z];
//            int i = windowLine.size();
//            p2iNewPointEdgDetectn = windowLine[0];
//            cout<<"\n window line end "<<p2iNewPointEdgDetectn;
            mapPointToRefer = mapPoint1;
        }
//        cout<<"\n point to which window is to be moved "<<p2iNewPointForWindow;
//        cout<<"\n ref point for edge detection "<<p2iNewPointEdgDetectn;
//        if(GetOppositEdgeOfPixelEdge(p2iNewPointEdgDetectn,newEdgeOfWindow)!= -1)
           {
            CheckWindowAroundAPixel(p2iNewPointForWindow,lineBorder,digLinesFound);
            if(mapCannyPixelPartOfLine[mapPointToRefer] == true)
                    break;//||
//               (CheckWindowAroundAPixel(p2iNewPointForWindow,newEdgeOfWindow,digLinesFound)== 1))
//                break;
        }

//        if((lineBorder == Left)||(lineBorder == Top))
//        {
//            if((lineTemp.p2iEnd1.x == (iImageWidth-1))||(lineTemp.p2iEnd2.x == (iImageWidth-1))||
//                (lineTemp.p2iEnd1.y == (iImageHeight-1))||(lineTemp.p2iEnd2.y == (iImageHeight-1)))
//                break;
//        }
//        else if((lineTemp.p2iEnd1.y == 0)||(lineTemp.p2iEnd2.y == 0)||
//                (lineTemp.p2iEnd1.x == 0)||(lineTemp.p2iEnd2.x == 0))
//            break;
//        if(digLinesFound.size()==iLinesFoundCount)
//        {
//            break;

//        }
    }
    }
//    cout<<"\n exited CheckWindowAroundAPixel ";
//    if(digLinesFound.size()==iLinesFoundCount)
//        return 1;
//    else
        return 0;

}
int Line_Detector_On_Canny::DetectLinesAlongBorder(Border lineBorder,vector<Line> &vLinesDetectedOnBorders)
{
    vector<cv::Point2i> vp2iPixels;
    vector<Line> vLinesDetected;
    Line strBorderLine;
    int iFixedPixel;
    bool isLineVertical;
    switch(lineBorder)
    {
        case Left:
            iFixedPixel = 0;
            isLineVertical = true;
            break;
        case Right:
            iFixedPixel = iImageWidth-1;
            isLineVertical = true;
            break;
        case Top:
            iFixedPixel = 0;
            isLineVertical = false;
            break;
        case Bottom:
            iFixedPixel = iImageHeight-1;
            isLineVertical = false;
            break;
    }

    if(isLineVertical)
    {
        strBorderLine.p2iEnd1.x = iFixedPixel;
        strBorderLine.p2iEnd2.x = iFixedPixel;
        strBorderLine.p2iEnd1.y = 0;
        strBorderLine.p2iEnd2.y = iImageHeight;
    }
    else
    {
        strBorderLine.p2iEnd1.y = iFixedPixel;
        strBorderLine.p2iEnd2.y = iFixedPixel;
        strBorderLine.p2iEnd1.x = 0;
        strBorderLine.p2iEnd2.x = iImageWidth;

    }
    DetectPixelsAlongALine(strBorderLine,isLineVertical,vp2iPixels);
    cout<<"\n No of Pixels found along border"<<vp2iPixels.size();
    cv::Mat matTempImage;

    cv::cvtColor(matCannyImage,matTempImage,CV_GRAY2BGR);

//    cv::line(matTempImage,strBorderLine.p2iEnd1,strBorderLine.p2iEnd2,cv::Scalar(0,255,0),1,8,0);
    for(int i = 0; i < vp2iPixels.size();i++)
    {
        //circle(matTempImage,vp2iPixels[i],5,cv::Scalar(255,0,0),2,8,0);
//        cout<<"\n moved window around border pixel "<<vp2iPixels[i];
        CheckWindowAroundAPixel(vp2iPixels[i],lineBorder,vLinesDetected);
    }
    for(int i = 0; i < vLinesDetected.size();i++)
    {
       Line lineTemp = vLinesDetected[i];
        cv::line(matTempImage,lineTemp.p2iEnd1,lineTemp.p2iEnd2,cv::Scalar(255,0,0),1,8,0);
        vLinesDetectedOnBorders.push_back(lineTemp);
    }
    cout<<"Lines detected on border "<<vLinesDetected.size();
//    cv::imshow(" lines found on a border ",matTempImage);
//    cv::waitKey();
//    string strImageName = strResultpath + "//LinesDetectedOnCanny.jpg";
//    cv::imwrite(strImageName,matTempImage);


}
int Line_Detector_On_Canny::GetPixelsOnLine(cv::Point2i p2iEnd1, cv::Point2i p2iEnd2, RefDigitalLine& RefDigLine)
{
    RefDigitalLine RefDigLineTemp;
    cv::Vec4i v4iLine;
    cv::Point2i p2iTempLine;
    double dSlope, dOffset;
    v4iLine[0] = p2iEnd1.x;
    v4iLine[1] = p2iEnd1.y;
    v4iLine[2] = p2iEnd2.x;
    v4iLine[3] = p2iEnd2.y;
    Get_Lines_A_B(v4iLine,dSlope,dOffset);
    if(p2iEnd1.x < p2iEnd2.x)
    {
        for(int i = p2iEnd1.x ; i <=p2iEnd2.x; i++)
        {
            p2iTempLine.x = i;
            p2iTempLine.y = (i*dSlope) + dOffset;
            RefDigLineTemp.push_back(p2iTempLine);
        }
    }
    if(RefDigLineTemp.size() > (iWindowSize/2))
    {
        RefDigLine = RefDigLineTemp;
        return 1;
    }
    else
    {
        return 0;
    }


}
int Line_Detector_On_Canny::CreateWindow()
{
    vector<RefDigitalLine> vRefWindowLines;
    RefDigitalLine strRefLineLeft, strRefLineRight, strRefLineTop, strRefLineBottom;
    cv::Point2i p2iPointLeftLine, p2iPointRightLine, p2iPointTopLine, p2iPointBottomLine, p2iPointLeftRight, p2iPointTopBottom;
    p2iPointLeftLine.x = 0;
    p2iPointRightLine.x = iWindowSize-1;
    p2iPointTopLine.y = 0;
    p2iPointBottomLine.y = iWindowSize-1;

    //create window border points
    for(int i =0; i < iWindowSize; i++)
    {
        p2iPointLeftLine.y = i;
        strRefLineLeft.push_back(p2iPointLeftLine);
        p2iPointRightLine.y = i;
        strRefLineRight.push_back(p2iPointRightLine);
        p2iPointTopLine.x = i;
        strRefLineTop.push_back(p2iPointTopLine);
        p2iPointBottomLine.x = i;
        strRefLineBottom.push_back(p2iPointBottomLine);
    }
    refWindowLeftLine = strRefLineLeft;
    refWindowRightLine = strRefLineRight;
    refWindowTopLine = strRefLineTop;
    refWindowBottomLine = strRefLineBottom;
    //create ref lines across all pixels of border lines

    //create lines between points on
    //left and top line and left and bottom of window
    //right and top line and right and bottom of window
    //left and right line
    for(int i =0; i < iWindowSize; i++)
    {
        RefDigitalLine refDigLineLeftRight;
        p2iPointLeftLine = strRefLineLeft[i];
        p2iPointRightLine = strRefLineRight[i];
        p2iPointLeftRight.y = p2iPointLeftLine.y;

        for(int j =0; j < iWindowSize; j++)
        {
            RefDigitalLine refLine;
            p2iPointTopLine = strRefLineTop[j];
            p2iPointBottomLine = strRefLineBottom[j];
            if(GetPixelsOnLine(p2iPointLeftLine, p2iPointTopLine,refLine)==1)//left-top
                vRefWindowLines.push_back(refLine);
            if(GetPixelsOnLine(p2iPointRightLine, p2iPointTopLine,refLine)==1)//right-top
                vRefWindowLines.push_back(refLine);
            if(GetPixelsOnLine(p2iPointLeftLine, p2iPointBottomLine,refLine)==1)//left-bottom
                vRefWindowLines.push_back(refLine);
            if(GetPixelsOnLine(p2iPointRightLine, p2iPointBottomLine,refLine)==1)//right-bottom
                vRefWindowLines.push_back(refLine);
            p2iPointLeftRight.x = j;
            refDigLineLeftRight.push_back(p2iPointLeftRight);
            p2iPointTopBottom.x = p2iPointTopLine.x;
            RefDigitalLine refDigLineTopBottom;
            for(int k=0; k < iWindowSize; k++)
            {
                p2iPointTopBottom.y = k;
                refDigLineTopBottom.push_back(p2iPointTopBottom);
            }
            vRefWindowLines.push_back(refDigLineTopBottom);//top-bottom

        }
        vRefWindowLines.push_back(refDigLineLeftRight);//left-right

    }

    refWindow = vRefWindowLines;

}
int Line_Detector_On_Canny::Initialise(int iWindowSizeInput, int iCorrelationPercentInput)
{
    iWindowSize = iWindowSizeInput;
    iCorrelationPercentage = iCorrelationPercentInput;
    iImageWidth = matCannyImage.size().width;
    iImageHeight = matCannyImage.size().height;
    ReadEdgePixels();
    CreateWindow();
}
int Line_Detector_On_Canny::DetectLinesAroundCannyPixels(vector<Line>& vlinesOnCanny)
{
    cv::Point2i p2iCannyPoint;

    map<pair<int,int>, bool>::iterator itCannyMap = mapCannyPixelPartOfLine.begin();
    map<pair<int,int>, bool>::iterator itCannyMapEnd = mapCannyPixelPartOfLine.end();
//    cout<<"\n in DetectLinesAroundCannyPixels";
    for(;itCannyMap != itCannyMapEnd;itCannyMap++)
    {
        if(itCannyMap->second == false)
        {
            p2iCannyPoint.x = (itCannyMap->first).first;
            p2iCannyPoint.y = (itCannyMap->first).second;
            if((p2iCannyPoint.x > (iImageWidth-1))||(p2iCannyPoint.y > (iImageHeight-1)))
            {
                break;
            }
//            cout<<"\n in DetectLinesAroundCannyPixels at "<<p2iCannyPoint;

            CheckWindowAroundAPixel(p2iCannyPoint,NotOnBorder,vlinesOnCanny);

        }
    }
}
int Line_Detector_On_Canny::DetectLines(vector<Line>& vlinesOnCanny )
{
    cv::Mat matTempImage;
    int iNoOfLinesDetected;
    cv::cvtColor(matCannyImage,matTempImage,CV_GRAY2BGR);
    bool flgColorBlue, flgColorGreen, flgColorRed;
    flgColorBlue = false;
    flgColorGreen = false;
    flgColorRed = false;
//    cout<<"\n before DetectLinesAlongBorders ";
//    DetectLinesAlongBorders(vlinesOnCanny);
    cout<<"\n detecting lines on non border pixels ";
    DetectLinesAroundCannyPixels(vlinesOnCanny);
    iNoOfLinesDetected = vlinesOnCanny.size();
    cout<<"\n No of Lines detected "<<iNoOfLinesDetected;
    for(int i = 0; i < iNoOfLinesDetected;i++)
    {
       Line lineTemp = vlinesOnCanny[i];
       cv::line(matTempImage,lineTemp.p2iEnd1,lineTemp.p2iEnd2,cv::Scalar(250,222,10),1,8,0);
    }
//    cv::imshow("total lines found",matTempImage);
//    cv::waitKey();
    string strImageName = strResultpath + "//LinesDetectedOnCanny.jpg";
    cv::imwrite(strImageName,matTempImage);
}
int Line_Detector_On_Canny::DetectLinesAlongBorders(vector<Line>& vLinesDetected)
{
//    vector<Line> vLinesDetected;
    cv::Vec2i v2iBorderLineEnds;
    cv::Mat matTempImage;

    cv::cvtColor(matCannyImage,matTempImage,CV_GRAY2BGR);

    //Detect pixels along left vertical line
    DetectLinesAlongBorder(Left,vLinesDetected);
//    Detect pixels along right vertical line
    DetectLinesAlongBorder(Right,vLinesDetected);
//    Detect pixels along top horizontal line
    DetectLinesAlongBorder(Top,vLinesDetected);
//    Detect pixels along bottom horizontal line
//    DetectLinesAlongBorder(Bottom,vLinesDetected);
//    for(int i = 0; i < vLinesDetected.size();i++)
//    {
//       Line lineTemp = vLinesDetected[i];
//       cv::line(matTempImage,lineTemp.p2iEnd1,lineTemp.p2iEnd2,cv::Scalar(255,0,0),1,8,0);
//    }
//    cv::imshow("lines found on borders",matTempImage);
//    cv::waitKey(2);
    cout<<"no. of line detected on border "<<vLinesDetected.size();
    return 1;
}

bool Line_Detector_On_Canny::DetectPixelsAlongALine(Line strBorderLine, bool flgIsLineVertical,vector<cv::Point2i>& vp2iPixels)
{
    uchar ucColor;
    bool flgPixelFound = false;
    cv::Point2i p2iPixel;
    if(flgIsLineVertical)
    {
        p2iPixel.x = strBorderLine.p2iEnd1.x;
        for(int i = strBorderLine.p2iEnd1.y; i < strBorderLine.p2iEnd2.y; i++)
        {
            p2iPixel.y = i;
            ucColor = matCannyImage.at<uchar>(p2iPixel.y,p2iPixel.x);

            if(ucColor== 255)
            {
                flgPixelFound = true;
                vp2iPixels.push_back(p2iPixel);
//                cout<<"\n found white pixel at: "<<p2iPixel;
            }
        }
    }
    else
    {
        p2iPixel.y = strBorderLine.p2iEnd1.y;
        for(int i = strBorderLine.p2iEnd1.x; i < strBorderLine.p2iEnd2.x; i++)
        {
            p2iPixel.x = i;
            ucColor = matCannyImage.at<uchar>(p2iPixel.y,p2iPixel.x);

            if(ucColor== 255)
            {
                flgPixelFound = true;
                vp2iPixels.push_back(p2iPixel);
//                cout<<"\n found white pixel at: "<<p2iPixel;
            }
        }

    }
    return flgPixelFound;

}
